import gym
from math import ceil
from gym import spaces
import numpy as np
from config import cfg
import requests

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
    def step(self):
        res = requests.get(f"{BOT_NAME}.local:5000/step")

    def reset(self):
        res = requests.get(f"{BOT_NAME}.local:5000/reset")