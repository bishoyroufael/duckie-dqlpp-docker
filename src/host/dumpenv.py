import gym
from math import ceil
from gym import spaces
import numpy as np
from config import cfg, BOT_NAME
import requests
import json
from requests.adapters import HTTPAdapter
from requests.packages.urllib3.util.retry import Retry

class DumpEnv(gym.Env):
    def __init__(self):
        self.depth_block_dim = ceil(cfg.depth_dim / cfg.block_size) 
        self.depth_array_min =  np.zeros((self.depth_block_dim, self.depth_block_dim), dtype=cfg.dtype)
        self.depth_array_max = np.ones((self.depth_block_dim, self.depth_block_dim), dtype=cfg.dtype)
        self.depth_array = self.depth_array_min

        self.action_space = spaces.Discrete(cfg.num_discrete_actions)

        self.observation_space = spaces.Box(
            low=self.depth_array_min, high=self.depth_array_max, dtype=cfg.dtype
        )

        self.headers = requests.utils.default_headers()

        self.headers.update(
            {
                'User-Agent': "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/51.0.2704.103 Safari/537.36",
            }
        )


        # self.session = requests.Session()
        # self.retry = Retry(connect=20, backoff_factor=0.5)
        # self.adapter = HTTPAdapter(max_retries=self.retry)
        # self.session.mount('http://', self.adapter)
        # self.session.mount('https://', self.adapter)

    def step(self, action):
        res = requests.post(f"http://{BOT_NAME}.local:5000/step?action={action}", headers=self.headers) 
        assert (res.status_code == 200) , f"[err] api return {res.status_code}"
        dic = json.loads(res.text)
        depth_array = np.array(dic["depth_array"], dtype=cfg.dtype).reshape(self.depth_array.shape)
        reward = dic['reward']
        terminate = dic['terminate']
        info = dic['info']
        info['tof_range'] = np.array(info['tof_range'], dtype=cfg.dtype)

        return depth_array, reward, terminate, info


    def reset(self):
        res = requests.get(f"http://{BOT_NAME}.local:5000/reset", headers=self.headers)
        dic = json.loads(res.text)
        observation = np.array(dic["observation"], dtype=cfg.dtype).reshape(self.depth_array.shape)
        return observation


# Test Environment
# if __name__ == "__main__":
#     env = DumpEnv()
#     observation = env.reset()
#     total_reward = 0

#     for _ in range(3):
#         while True:
#             action = env.action_space.sample()
#             # print(action)
#             obs, reward, done, info = env.step(action)
#             print(obs.var())
#             total_reward = total_reward + reward
#             if done:
#                 break

#         print(f"total_reward in eps {_} : {total_reward} ")
#         total_reward = 0
