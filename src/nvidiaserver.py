import rospy
from config import cfg
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from utils import ImgUtils, NvidiaUtils

'''
	This node is meant to estimate depth and publish it on /raw_depth
	The gym environment should use the raw depth as an observation
'''


nvidia_utils = NvidiaUtils()

msg_depth = Float32MultiArray()
msg_depth.layout.dim.append(MultiArrayDimension())

pub = rospy.Publisher(cfg.raw_depth_topic, Float32MultiArray, queue_size=10)
def _callback(msg):
	global msg_depth
	global pub
	depth_array = nvidia_utils.compute_depth_array(ImgUtils.rosimg_to_opencv(msg.data))
	depth_array_normalized = (depth_array - np.min(depth_array, axis=0)) / np.ptp(depth_array, axis=0) 
	
	msg_depth.data = depth_array_normalized.ravel().tolist()
	pub.publish(msg_depth)

rospy.init_node('nvidia_node', anonymous = True)
rospy.Subscriber(cfg.camera_topic, CompressedImage, _callback)

rospy.spin()
