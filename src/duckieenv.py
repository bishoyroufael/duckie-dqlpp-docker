import gym
from gym import spaces
import rospy
from config import cfg
import numpy as np
from sensor_msgs.msg import Range
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Float32MultiArray
import time
import torch

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
        rospy.Subscriber(cfg.raw_depth_topic, Float32MultiArray , self.raw_depth_callback)
        self.speed_pub = rospy.Publisher(cfg.wheels_topic, WheelsCmdStamped, queue_size=10) 

        # Gym environment variables
        self.min_action = np.array([0.0, 0.0], dtype=np.float32) 
        self.max_action = np.array([cfg.max_bot_delta_speed, cfg.max_bot_delta_speed], dtype=np.float32) 

        # ToF reading for terminating an episode 
        self.min_tof_dist = np.array([cfg.tof_min_range], dtype=np.float32) # m
        self.max_tof_dist = np.array([cfg.tof_max_range], dtype=np.float32) # m
        self.tof_range = self.min_tof_dist

        self.depth_array_min =  np.zeros((cfg.depth_dim * cfg.depth_dim), dtype=np.float32)
        self.depth_array_max = np.ones((cfg.depth_dim * cfg.depth_dim), dtype=np.float32)
        self.depth_array = self.depth_array_min


        # Action is rotation
        self.action_space = spaces.Box(
            low=self.min_action, high=self.max_action, dtype=np.float32
        )

        self.observation_space = spaces.Box(
            low=self.depth_array_min, high=self.depth_array_max, dtype=np.float32
        )

        # print("Sleeping 5 secs for sensors initialization to take place")
        # time.sleep(5)

    def tof_callback(self, msg):
        # Clip if below or above limits
        if msg.range < 0 :
            range  = 0
        elif msg.range > 1.2:
            range = 1.2
        else:
            range = msg.range 
        self.tof_range = np.array([range], dtype=np.float32)


    def raw_depth_callback(self, msg):
        self.depth_array = np.array(msg.data) 


    def reset(self):
        self.observation_space = self.depth_array_min 
        self.stop_bot()
        return self.observation_space

    def stop_bot(self):
        self.speed_pub.publish(None, 0, 0)

    def move(self, action : np.ndarray):
        self.speed_pub.publish(None, cfg.bot_speed + action[0] , cfg.bot_speed + action[1] )

    def rotate_slowly(self):
        self.speed_pub.publish(None, cfg.rotation_speed, 0)

    def reposition(self):
        # Go Backward until it's safe to rotate
        while(self.tof_range <= (self.max_tof_dist.item()/6)):
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

        terminate = False
        assert self.action_space.contains(action), "[err] action is invalid"

        reward = 1

        # Encourge straight motion
        if action[0] == 0 and action[1] == 0:
            reward = reward + 0.2

        self.move(action)

        # 0 <= mean <= 1
        # print(f"Depth array mean: {np.mean(self.depth_array)}")

        if self.tof_range <= cfg.tof_min_range_threshold:
            terminate = True
            reward = -100
            self.stop_bot()
            self.reposition()

        return self.depth_array, reward, terminate, {"tof_range": self.tof_range}



# Test Environment
if __name__ == "__main__":
    env = DuckieEnv()
    observation = env.reset()
    total_reward = 0


    for _ in range(3):
        while True:
            action = env.action_space.sample()
            # print(action)
            obs, reward, done, info = env.step(action)
            total_reward = total_reward + reward
            if done:
                break

        print(f"total_reward in eps {_} : {total_reward} ")
        total_reward = 0




