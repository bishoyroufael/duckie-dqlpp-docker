import argparse
import os
import pprint
from duckieenv import DuckieEnv
from config import cfg, BOT_NAME
import numpy as np
import torch
from torch.utils.tensorboard import SummaryWriter

from tianshou.data import Collector, VectorReplayBuffer
from tianshou.env import DummyVectorEnv
from tianshou.exploration import GaussianNoise
from tianshou.policy import DDPGPolicy
from tianshou.trainer import offpolicy_trainer
from tianshou.utils import TensorboardLogger
from tianshou.utils.net.common import Net
from tianshou.utils.net.continuous import Actor, Critic



def test_ddpg():
    env =  DuckieEnv()
    state_shape = env.observation_space.shape or env.observation_space.n
    action_shape = env.action_space.shape or env.action_space.n
    max_action = env.action_space.high[0]
    print(state_shape, action_shape, max_action)

    train_envs = DummyVectorEnv(
        [lambda: DuckieEnv() for _ in range(cfg.training.training_num)]
    )
    # test_envs = gym.make(cfg.trainingtask)
    test_envs = DummyVectorEnv(
        [lambda: DuckieEnv() for _ in range(cfg.training.test_num)]
    )
    # seed
    np.random.seed(cfg.training.seed)
    torch.manual_seed(cfg.training.seed)
    train_envs.seed(cfg.training.seed)
    test_envs.seed(cfg.training.seed)

    print("Getting Model")
    # model
    net = Net(state_shape, hidden_sizes=cfg.training.hidden_sizes, device=cfg.training.device)
    actor = Actor(
        net, action_shape, max_action=max_action, device=cfg.training.device
    ).to(cfg.training.device)
    actor_optim = torch.optim.Adam(actor.parameters(), lr=cfg.training.actor_lr)
    net = Net(
        state_shape,
        action_shape,
        hidden_sizes=cfg.training.hidden_sizes,
        concat=True,
        device=cfg.training.device
    )
    critic = Critic(net, device=cfg.training.device).to(cfg.training.device)
    critic_optim = torch.optim.Adam(critic.parameters(), lr=cfg.training.critic_lr)

    print("Models loaded")
    policy = DDPGPolicy(
        actor,
        actor_optim,
        critic,
        critic_optim,
        tau=cfg.training.tau,
        gamma=cfg.training.gamma,
        exploration_noise=GaussianNoise(sigma=cfg.training.exploration_noise),
        reward_normalization=cfg.training.rew_norm,
        estimation_step=cfg.training.n_step,
        action_space=env.action_space
    )
    # collector
    train_collector = Collector(
        policy,
        train_envs,
        VectorReplayBuffer(cfg.training.buffer_size, len(train_envs)),
        exploration_noise=True
    )
    test_collector = Collector(policy, test_envs)
    # log
    log_path = os.path.join(cfg.training.logdir, BOT_NAME, 'ddpg')
    writer = SummaryWriter(log_path)
    logger = TensorboardLogger(writer)

    def save_best_fn(policy):
        torch.save(policy.state_dict(), os.path.join(log_path, 'policy.pth'))

    def stop_fn(mean_rewards):
        return mean_rewards >= cfg.training.reward_threshold

    # trainer
    result = offpolicy_trainer(
        policy,
        train_collector,
        test_collector,
        cfg.training.epoch,
        cfg.training.step_per_epoch,
        cfg.training.step_per_collect,
        cfg.training.test_num,
        cfg.training.batch_size,
        update_per_step=cfg.training.update_per_step,
        stop_fn=stop_fn,
        save_best_fn=save_best_fn,
        logger=logger
    )
    assert stop_fn(result['best_reward'])

    if __name__ == '__main__':
        pprint.pprint(result)
        # Let's watch its performance!
        env = DuckieEnv()
        policy.eval()
        collector = Collector(policy, env)
        result = collector.collect(n_episode=1)
        rews, lens = result["rews"], result["lens"]
        print(f"Final reward: {rews.mean()}, length: {lens.mean()}")


if __name__ == '__main__':
    test_ddpg()

