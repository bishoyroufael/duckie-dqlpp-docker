import cv2
import numpy as np


class ApiUtils:
    # Method for getting the video buffer to flask api
    @staticmethod
    def get_stream_buffer(feed, depth=False):
        # feed : VideoFeed instance
        while True:
            if depth:
                depth_image = feed.get_depth_img()
            retval, buffer = cv2.imencode('.png', feed.get_rgb_img()) if not depth else cv2.imencode('.png', depth_image)
            yield (b'--frame\r\n' b'Content-Type: image/png\r\n\r\n' + buffer.tobytes() + b'\r\n\r\n')