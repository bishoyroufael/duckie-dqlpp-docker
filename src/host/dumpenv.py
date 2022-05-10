import gym
from math import ceil
from gym import spaces
import numpy as np
from config import cfg, BOT_NAME
import requests
import json
import matplotlib.pyplot as plt
import cv2
import io

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

        self.headers = requests.utils.default_headers()

        self.headers.update(
            {
                'User-Agent': "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/51.0.2704.103 Safari/537.36",
            }
        )

    def step(self, action):
        res = requests.post(f"http://{BOT_NAME}.local:5000/step?action={action}", headers=self.headers) 
        assert (res.status_code == 200) , f"[err] api return {res.status_code}"
        res = np.load(io.BytesIO(res.content))
        depth_array = res ['arr_0']
        reward = res ['arr_1'].item()
        terminate = bool(res ['arr_2'].item())
        info = {"tof_range":res ['arr_3']}
      
        return depth_array, reward, terminate, info


    def reset(self):
        res = requests.get(f"http://{BOT_NAME}.local:5000/reset", headers=self.headers)
        res = np.load(io.BytesIO(res.content))
        observation = res['arr_0'] 
        return observation


# Test Environment
if __name__ == "__main__":
    env = DumpEnv()
    observation = env.reset()
    total_reward = 0
    DEPTH_ONLY = True
    
    if not DEPTH_ONLY:
        for _ in range(3):
            while True:
                action = env.action_space.sample()
                # print(action)
                obs, reward, done, info = env.step(action)
                # Show depth
                total_reward = total_reward + reward
                if done:
                    break

            print(f"total_reward in eps {_} : {total_reward} ")
            total_reward = 0


    else:
        while True:
            obs, _ , _, _ = env.step(-1)
            # obs *= 255
            # cv2.imshow("raw depth", obs.astype(np.uint8))
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break
            plt.imshow(obs, cmap="gray")
            plt.pause(0.01)
            plt.draw()
