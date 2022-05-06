# https://stackoverflow.com/questions/16780014/import-file-from-parent-directory
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import cfg
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray


def _callback(msg):
	print(f"depth variance: {np.array(msg.data).reshape((cfg.depth_dim, cfg.depth_dim)).var()}")


rospy.init_node('test', anonymous= True)
rospy.Subscriber('/raw_depth', Float32MultiArray, _callback)
rospy.spin()