import cv2
import jetson.inference
import jetson.utils
from depthnet_utils import depthBuffers
from argparse import Namespace
import numpy as np

class ImgUtils:
    @staticmethod
    def rosimg_to_opencv(data):
        np_arr = np.fromstring(data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        return image_np

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


# Class for handling nvidia depth predictions
# There should be ONE instance of that class in the main script!
class NvidiaUtils:
    def __init__(self):
        self.args = {"depth_size" : 0.5, "visualize": "depth"}
        self.buffers = depthBuffers( Namespace(**self.args) )

        # load mono depth network
        self.net = jetson.inference.depthNet("fcn-mobilenet", ['--log-level=error'])

        # depthNet re-uses the same memory for the depth field,
        # so you only need to do this once (not every frame)
        self.depth_field = self.net.GetDepthField()

        # cudaToNumpy() will map the depth field cudaImage to numpy
        # this mapping is persistent, so you only need to do it once
        self.depth_numpy = jetson.utils.cudaToNumpy(self.depth_field)

        print(f"depth field resolution is {self.depth_field.width}x{self.depth_field.height}")

    def get_depth_field_dims(self):
        return (self.depth_field.width, self.depth_field.height)


    def compute_depth_array(self, img):
        cuda_arr =  jetson.utils.cudaFromNumpy(img)
        self.net.Process(cuda_arr)
        jetson.utils.cudaDeviceSynchronize() # wait for GPU to finish processing, so we can use the results on CPU
        return self.depth_numpy


    def compute_depth_image(self, img):
        
        cuda_arr =  jetson.utils.cudaFromNumpy(img)
        self.buffers.Alloc(cuda_arr.shape, cuda_arr.format)

        self.net.Process(cuda_arr, self.buffers.depth, 'turbo' , 'point' )

        # composite the images
        if self.buffers.use_input:
            jetson.utils.cudaOverlay(cuda_arr, self.buffers.composite, 0, 0)
            
        if self.buffers.use_depth:
            jetson.utils.cudaOverlay(self.buffers.depth, self.buffers.composite, cuda_arr.width if self.buffers.use_input else 0, 0)

        numpy_composite = jetson.utils.cudaToNumpy(self.buffers.composite)

        jetson.utils.cudaDeviceSynchronize() # wait for GPU to finish processing, so we can use the results on CPU
        return numpy_composite
