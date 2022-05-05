import gym
from gym import spaces
import rospy
from config import cfg
import numpy as np
from sensor_msgs.msg import CompressedImage, Range
from duckietown_msgs.msg import WheelsCmdStamped
from utils import ImgUtils, NvidiaUtils

class DuckieEnv(gym.Env):
    '''
        Action space : velocities of the 2 wheels in duckiebot 
        Observation space : ToF distance reading 
    '''
    def __init__(self):
        
        rospy.set_param(cfg.camera_param_res[0], cfg.camera_width) # width
        rospy.set_param(cfg.camera_param_res[1], cfg.camera_height) # height

        # self.nvidia_utils = NvidiaUtils()

        rospy.init_node('rl_obs', anonymous=True)
        rospy.Subscriber(cfg.camera_topic, CompressedImage, self.image_callback)
        rospy.Subscriber(cfg.tof_topic, Range, self.tof_callback)

        speed_pub = rospy.Publisher(cfg.wheels_topic, WheelsCmdStamped, queue_size=10) 

        self.min_action = np.array([-0.2, -0.2], dtype=np.float32) 
        self.max_action = np.array([0.2, 0.2], dtype=np.float32) 
        self.min_tof_dist = np.array([0.0], dtype=np.float32) # m
        self.max_tof_dist = np.array([1.2], dtype=np.float32) # m
        self.depth_array = np.zeros((244,244), dtype=np.float32)        
        self.tof_range = self.min_tof_dist


        self.action_space = spaces.Box(
            low=self.min_action, high=self.max_action, shape=(2,), dtype=np.float32
        )

        self.observation_space = spaces.Box(
            low=self.min_tof_dist, high=self.max_tof_dist, dtype=np.float32
        )

    def tof_callback(self, msg):
        # Clip if below or above limits
        if msg.range < 0 :
            range  = 0
        elif msg.range > 1.2:
            range = 1.2
        else:
            range = msg.range 
        self.tof_range = np.array([range], dtype=np.float32)

    def image_callback(self, msg):
        # self.depth_array = self.nvidia_utils.compute_depth_array(ImgUtils.rosimg_to_opencv(msg.data)) 
        # self.depth_array = (self.depth_array - np.amin(self.depth_array)) / (np.amax(self.depth_array) - np.amin(self.depth_array))
        pass

    def reset(self):
        self.ep_reward = 0

    def step(self, action: np.ndarray):
        pass



env = DuckieEnv()
print(env.observation_space)
rospy.spin()





