import rospy
from config import cfg
import numpy as np
from sensor_msgs.msg import CompressedImage
from utils import ImgUtils, NvidiaUtils
import mmap

'''
	This node is meant to estimate depth and publish it on /raw_depth
	The gym environment should use the raw depth as an observation
	It uses shared-memory for fast IPC 
'''


nvidia_utils = NvidiaUtils()

f = np.memmap('raw_d_mm', dtype=cfg.dtype, mode='w+', shape=(cfg.depth_dim, cfg.depth_dim))
def _callback(msg):
	depth_array = nvidia_utils.compute_depth_array(ImgUtils.rosimg_to_opencv(msg.data))
	f[:] = depth_array.squeeze().astype(cfg.dtype)[:]

rospy.init_node('nvidia_node', anonymous = True)
rospy.Subscriber(cfg.camera_topic, CompressedImage, _callback)

rospy.spin()
