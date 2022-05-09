import argparse
import os
import pprint
import numpy as np
import torch
from torch.utils.tensorboard import SummaryWriter
from tianshou.data import Collector, VectorReplayBuffer
from tianshou.env import DummyVectorEnv
from tianshou.policy import DQNPolicy
from tianshou.trainer import offpolicy_trainer
from tianshou.utils import TensorboardLogger
from tianshou.utils.net.common import Net
from duckieenv import DuckieEnv
from config import cfg, BOT_NAME
import pickle

def run_dqn():
    env = DuckieEnv() 
    state_shape = env.observation_space.shape or env.observation_space.n
    action_shape = env.action_space.shape or env.action_space.n


    train_envs = DummyVectorEnv(
        [lambda: DuckieEnv() for _ in range(cfg.training.dqn.training_num)]
    )
    test_envs = DummyVectorEnv(
        [lambda: DuckieEnv() for _ in range(cfg.training.dqn.test_num)]
    )
    # seed
    np.random.seed(cfg.training.dqn.seed)
    torch.manual_seed(cfg.training.dqn.seed)
    train_envs.seed(cfg.training.dqn.seed)
    test_envs.seed(cfg.training.dqn.seed)
    # Q_param = V_param = {"hidden_sizes": [128]}
    # model

    print("[info] Building DQN Model")
    net = Net(
        state_shape,
        action_shape,
        hidden_sizes=cfg.training.dqn.hidden_sizes,
        device=cfg.training.dqn.device,
    ).to(cfg.training.dqn.device)

    optim = torch.optim.Adam(net.parameters(), lr=cfg.training.dqn.lr)

    policy = DQNPolicy(
        net,
        optim,
        cfg.training.dqn.gamma,
        cfg.training.dqn.n_step,
        target_update_freq=cfg.training.dqn.target_update_freq,
    )
    # buffer
    buf = VectorReplayBuffer(cfg.training.dqn.buffer_size, buffer_num=len(train_envs))


    # collector
    train_collector = Collector(policy, train_envs, buf, exploration_noise=True)
    test_collector = Collector(policy, test_envs, exploration_noise=True)
    
    # policy.set_eps(1)
    train_collector.collect(n_step=cfg.training.dqn.batch_size * cfg.training.dqn.training_num)
    
    
    # log
    log_path = os.path.join(cfg.training.dqn.logdir, BOT_NAME , 'dqn')
    writer = SummaryWriter(log_path)
    logger = TensorboardLogger(writer)

    def save_best_fn(policy):
        torch.save(policy.state_dict(), os.path.join(log_path, 'policy.pth'))

    def stop_fn(mean_rewards):
        return mean_rewards >= cfg.training.dqn.reward_threshold

    def train_fn(epoch, env_step):
        # eps annnealing, just a demo
        if env_step <= 10000:
            policy.set_eps(cfg.training.dqn.eps_train)
        elif env_step <= 50000:
            eps = cfg.training.dqn.eps_train - (env_step - 10000) / \
                40000 * (0.9 * cfg.training.dqn.eps_train)
            policy.set_eps(eps)
        else:
            policy.set_eps(0.1 * cfg.training.dqn.eps_train)

    def test_fn(epoch, env_step):
        policy.set_eps(cfg.training.dqn.eps_test)


    # Resumable Training 
    # https://github.com/thu-ml/tianshou/blob/master/test/discrete/test_c51.py
    def save_checkpoint_fn(epoch, env_step, gradient_step):
        # see also: https://pytorch.org/tutorials/beginner/saving_loading_models.html
        torch.save(
            {
                'model': policy.state_dict(),
                'optim': optim.state_dict(),
            }, os.path.join(log_path, 'checkpoint.pth')
        )
        pickle.dump(
            train_collector.buffer,
            open(os.path.join(log_path, 'train_buffer.pkl'), "wb")
        )

    if cfg.training.dqn.resume:
        # load from existing checkpoint
        print(f"Loading agent under {log_path}")
        ckpt_path = os.path.join(log_path, 'policy.pth')
        if os.path.exists(ckpt_path):
            checkpoint = torch.load(ckpt_path, map_location=cfg.training.device)
            policy.load_state_dict(checkpoint['model'])
            policy.optim.load_state_dict(checkpoint['optim'])
            print("Successfully restore policy and optim.")
        else:
            print("Fail to restore policy and optim.")
        buffer_path = os.path.join(log_path, 'train_buffer.pkl')
        if os.path.exists(buffer_path):
            train_collector.buffer = pickle.load(open(buffer_path, "rb"))
            print("Successfully restore buffer.")
        else:
            print("Fail to restore buffer.")


    print("[info] Done!")

    # trainer
    result = offpolicy_trainer(
        policy,
        train_collector,
        test_collector,
        cfg.training.dqn.epoch,
        cfg.training.dqn.step_per_epoch,
        cfg.training.dqn.step_per_collect,
        cfg.training.dqn.test_num,
        cfg.training.dqn.batch_size,
        update_per_step=cfg.training.dqn.update_per_step,
        train_fn=train_fn,
        test_fn=test_fn,
        stop_fn=stop_fn,
        save_best_fn=save_best_fn,
        logger=logger,
        save_checkpoint_fn = save_checkpoint_fn
    )

    if not stop_fn(result['best_reward']):
        print("[warn] Best reward wasn't achieved yet!")

    if __name__ == '__main__':
        pprint.pprint(result)
        # Let's watch its performance!
        env = DuckieEnv() 
        policy.eval()
        policy.set_eps(cfg.training.dqn.eps_test)
        collector = Collector(policy, env)
        result = collector.collect(n_episode=1, render=0.)
        rews, lens = result["rews"], result["lens"]
        print(f"Final reward: {rews.mean()}, length: {lens.mean()}")


if __name__ == '__main__':
    run_dqn()