import jetson.inference
import jetson.utils
from depthnet_utils import depthBuffers
from argparse import Namespace
import numpy as np

# load mono depth network
net = jetson.inference.depthNet("fcn-mobilenet")

# depthNet re-uses the same memory for the depth field,
# so you only need to do this once (not every frame)
depth_field = net.GetDepthField()

# cudaToNumpy() will map the depth field cudaImage to numpy
# this mapping is persistent, so you only need to do it once
depth_numpy = jetson.utils.cudaToNumpy(depth_field)

print(f"depth field resolution is {depth_field.width}x{depth_field.height}, format={depth_field.format})")


args = {"depth_size" : 0.5, "visualize": "depth"}
buffers = depthBuffers( Namespace(**args) )


def get_depth_field_dims():
    global depth_field
    return (depth_field.width, depth_field.height)

def compute_depth(img):
    global net
    global depth_numpy
    
    cuda_arr =  jetson.utils.cudaFromNumpy(img)
    buffers.Alloc(cuda_arr.shape, cuda_arr.format)

    net.Process(cuda_arr, buffers.depth, 'turbo' , 'point' )

    # composite the images
    if buffers.use_input:
        jetson.utils.cudaOverlay(cuda_arr, buffers.composite, 0, 0)
        
    if buffers.use_depth:
        jetson.utils.cudaOverlay(buffers.depth, buffers.composite, cuda_arr.width if buffers.use_input else 0, 0)

    numpy_composite = jetson.utils.cudaToNumpy(buffers.composite)

    jetson.utils.cudaDeviceSynchronize() # wait for GPU to finish processing, so we can use the results on CPU
    return numpy_composite


# import cv2
# x = cv2.imread('../testimg.png')
# y = compute_depth(x)
# print(y.shape)

