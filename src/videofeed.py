import cv2
import nvidia
import numpy as np

class VideoFeed:
    def __init__(self, W, H):
        self.rgb_img = np.zeros((W,H,3), dtype=np.uint8)
        self.depth_img = np.zeros((W,H,3), dtype=np.uint8)

    def __rosimg_to_opencv(self, data):
        np_arr = np.fromstring(data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        return image_np

    def get_depth_img(self):
        return self.depth_img

    def get_rgb_img(self):
        return self.rgb_img

    def update(self, msg):
        self.rgb_img = self.__rosimg_to_opencv(msg.data)
        self.depth_img = nvidia.compute_depth(self.rgb_img)  