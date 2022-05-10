import gym
from gym import spaces
import rospy
from config import cfg
import numpy as np
from sensor_msgs.msg import Range
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Float32MultiArray
import time
from math import ceil
from skimage.measure import block_reduce
from scipy import stats


class DuckieEnv(gym.Env):
    '''
        Action space : +ve delta velocities of the 2 wheels in duckiebot. (i.e robot always moving forward) 
        Observation space : depth array from nvidia inference node
    '''
    def __init__(self):
        
        # ROS Stuff
        rospy.set_param(cfg.camera_param_res[0], cfg.camera_width) # width
        rospy.set_param(cfg.camera_param_res[1], cfg.camera_height) # height
        rospy.init_node('rl_obs', anonymous=True)
        rospy.Subscriber(cfg.tof_topic, Range, self.tof_callback)
        try:
            self.mm = np.memmap('raw_d_mm', dtype=cfg.dtype, mode='r+', shape=(cfg.depth_dim,cfg.depth_dim))
        except FileNotFoundError:
            print("[warn] memory mapped file not found for depth estimation, please execute run_nvidia_mm.sh and try again!")
            exit(-1)

        self.speed_pub = rospy.Publisher(cfg.wheels_topic, WheelsCmdStamped, queue_size=10) 


        # ToF reading for terminating an episode 
        self.min_tof_dist = np.array([cfg.tof_min_range], dtype=cfg.dtype) # m
        self.max_tof_dist = np.array([cfg.tof_max_range], dtype=cfg.dtype) # m
        self.tof_range = self.min_tof_dist

        self.depth_block_dim = ceil(cfg.depth_dim / cfg.block_size) 
        self.depth_array_min =  np.zeros((self.depth_block_dim, self.depth_block_dim), dtype=cfg.dtype)
        self.depth_array_max = np.ones((self.depth_block_dim, self.depth_block_dim), dtype=cfg.dtype)
        self.depth_array = self.depth_array_min

        self.action_space = spaces.Discrete(cfg.num_discrete_actions)

        self.observation_space = spaces.Box(
            low=self.depth_array_min, high=self.depth_array_max, dtype=cfg.dtype
        )


    def tof_callback(self, msg):
        # Clip if below or above limits
        if msg.range < cfg.tof_min_range:
            range  = cfg.tof_min_range
        elif msg.range > cfg.tof_max_range:
            range = cfg.tof_max_range
        else:
            range = msg.range 
        self.tof_range = np.array([range], dtype=cfg.dtype)


    def update_observation(self):

        # block_reduced = block_reduce(self.mm, (cfg.block_size, cfg.block_size)) # Size : ceil(cfg.depth_dim / cfg.block_size)
        # self.depth_array =  (block_reduced- np.min(block_reduced, axis=0)) / np.ptp(block_reduced, axis=0) 
        
        self.depth_array = self.mm

    def reset(self):
        self.stop_bot()
        return self.depth_array_min

    def stop_bot(self):
        self.speed_pub.publish(None, 0, 0)

    def move(self, action : np.ndarray):
        if action == 0: # Forward
            self.speed_pub.publish(None, cfg.bot_speed, cfg.bot_speed)
        else:
            mid = (self.action_space.n // 2) 
            if action <= mid :
                delta =  (cfg.max_bot_delta_speed / (self.action_space.n - (2*action) ))
                self.speed_pub.publish(None, cfg.bot_speed +  delta  , cfg.bot_speed - delta)
            else:
                delta =  (cfg.max_bot_delta_speed / (self.action_space.n - (2*(action - mid))  ))
                self.speed_pub.publish(None, cfg.bot_speed -  delta  , cfg.bot_speed + delta)

        # self.speed_pub.publish(None, cfg.bot_speed + action[0] , cfg.bot_speed - action[0] )

    def rotate_slowly(self):
        self.speed_pub.publish(None, cfg.rotation_speed, -cfg.rotation_speed)

    def reposition(self):
        # Go Backward until it's safe to rotate
        while(self.tof_range <= (self.max_tof_dist.item()/4)):
            self.speed_pub.publish(None, -cfg.reverse_speed, -cfg.reverse_speed)

        # Rotate until there is plenty of space for motion
        while(self.tof_range < (self.max_tof_dist.item()/2)):
            self.rotate_slowly()

        self.stop_bot()

    def is_ready(self):
        # Check whether ToF readings started to appear
        if self.tof_range != self.min_tof_dist:
            return True
        return False


    def step(self, action):
        while True:
            if self.is_ready():
                break

        # to visualize depth without taking any action
        if (action == -1): 
            self.update_observation()
            return self.depth_array, 0, 0, {'tof_range': self.tof_range}

        terminate = False
        assert self.action_space.contains(action), "[err] action is invalid"
    
        reward = 1

        # Encourge straight motion
        if action == 0:
            reward = reward + 1
        # Discourage extreme rotation
        elif (action == self.action_space.n - 1) or (action == self.action_space.n // 2):
            reward = reward - 0.9

        # if action[0] >= cfg.max_bot_delta_speed * cfg.discourage_rate:
        #     reward = reward - 0.5 

        self.move(action)

        # 0 <= mean <= 1
        # print(f"Depth array mean: {np.mean(self.depth_array)}")

        if  self.tof_range <= cfg.tof_min_range_threshold:
            terminate = True
            reward = -100 
            self.stop_bot()
            self.reposition()

        self.update_observation() # Get depth observation stats (updates self.depth_array)
        return self.depth_array, reward, terminate, {"tof_range": self.tof_range}



# Test Environment
# if __name__ == "__main__":
#     env = DuckieEnv()
#     observation = env.reset()
#     total_reward = 0
#     for _ in range(1):
#         while True:
#             action = env.action_space.sample()
#             # print(action)
#             obs, reward, done, info = env.step(4)
#             # print(obs.var())
#             total_reward = total_reward + reward
#             if done:
#                 break

#         print(f"total_reward in eps {_} : {total_reward} ")
#         total_reward = 0




