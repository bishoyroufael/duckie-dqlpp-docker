# DEPRECATED 

import cv2
import numpy as np
from utils import NvidiaUtils

# Class for managing ROS images 
class VideoFeed:
    def __init__(self):
        self.n_utils = NvidiaUtils()
        self.rgb_img = np.zeros((200, 200,3), dtype=np.uint8)
        self.depth_img = np.zeros((200, 200,3), dtype=np.uint8)

    def __rosimg_to_opencv(self, data):
        np_arr = np.fromstring(data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        return image_np

    def get_depth_img(self):
        self.depth_img = self.n_utils.compute_depth_image(self.rgb_img)  
        return self.depth_img

    def get_depth_array(self):
        return self.n_utils.compute_depth_array(self.rgb_img)

    def get_rgb_img(self):
        return self.rgb_img

    def update(self, msg, get_raw_depth=False):
        self.rgb_img = self.__rosimg_to_opencv(msg.data)
        if get_raw_depth:
            return self.get_depth_array()