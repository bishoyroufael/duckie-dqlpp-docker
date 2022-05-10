import gym
from math import ceil
from gym import spaces
import numpy as np
from config import cfg, BOT_NAME
import requests
import json

class DumpEnv(gym.Env):
    def __init__(self):
        self.depth_block_dim = ceil(cfg.depth_dim / cfg.block_size) 
        self.depth_array_min =  np.zeros((self.depth_block_dim, self.depth_block_dim), dtype=np.float16)
        self.depth_array_max = np.ones((self.depth_block_dim, self.depth_block_dim), dtype=np.float16)
        self.depth_array = self.depth_array_min

        self.action_space = spaces.Discrete(cfg.num_discrete_actions)

        self.observation_space = spaces.Box(
            low=self.depth_array_min, high=self.depth_array_max, dtype=np.float16
        )
        pass

    def step(self, action):
        res = requests.post(f"http://{BOT_NAME}.local:5000/step?action={action}") 
        assert (res.status_code == 200) , f"[err] api return {res.status_code}"
        dic = json.loads(res.text)
        depth_array = np.array(dic["depth_array"], dtype=np.float16).reshape(self.depth_array.shape)
        reward = dic['reward']
        terminate = dic['terminate']
        info = dic['info']
        info['tof_range'] = np.array(info['tof_range'], dtype=np.float16)

        return depth_array, reward, terminate, info


    def reset(self):
        res = requests.get(f"http://{BOT_NAME}.local:5000/reset")
        dic = json.loads(res.text)
        observation = np.array(dic["observation"], dtype=np.float16).reshape(self.depth_array.shape)
        return observation


# Test Environment
if __name__ == "__main__":
    env = DumpEnv()
    observation = env.reset()
    total_reward = 0

    for _ in range(3):
        while True:
            action = env.action_space.sample()
            # print(action)
            obs, reward, done, info = env.step(action)
            # print(obs.var())
            total_reward = total_reward + reward
            if done:
                break

        print(f"total_reward in eps {_} : {total_reward} ")
        total_reward = 0
