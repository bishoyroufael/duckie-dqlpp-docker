import rospy
import flask
import numpy as np
from sensor_msgs.msg import CompressedImage
import cv2
from flask import app, Flask, jsonify, Response, render_template
from config import cfg
from utils import ApiUtils, NvidiaUtils
from videofeed import VideoFeed
from duckietown_msgs.msg import WheelsCmdStamped



rospy.set_param(cfg.camera_param_res[0], cfg.camera_width) # width
rospy.set_param(cfg.camera_param_res[1], cfg.camera_height) # height

# Store image buffers
feed = VideoFeed()

# Flask initialization
# app = Flask(__name__)
# app.debug = True


# Flask Endpoints
# @app.route('/rgb_video')
# def rgb_video():
    # return Response(ApiUtils.get_stream_buffer(feed), mimetype='multipart/x-mixed-replace; boundary=frame')

# @app.route('/depth_video')
# def depth_video():
#     return Response(ApiUtils.get_stream_buffer(feed, depth=True), mimetype='multipart/x-mixed-replace; boundary=frame')

# @app.route('/')
# def index():
#     return render_template('index.html')


i = 0
# Callback for subscriber
# Training happen here each time an image is recieved
def image_callback(msg):
    global i
    print(f"Received frame: {i}!")
    i = i + 1
    # speed_pub = rospy.Publisher(cfg.wheels_topic, WheelsCmdStamped, queue_size=10) 
    try:
        # speed_pub.publish(None, 0.0, 0.0)
        feed.update(msg) 
        depth = feed.get_depth_array()
        print(f"min : {np.amin(depth)} max: {np.amax(depth)} dim: {depth.shape}")

    except Exception as e:
        print(e)

if __name__ == '__main__':
    # app.run(host='0.0.0.0')
    rospy.init_node('deep_q_learning')
    rospy.Subscriber(cfg.camera_topic, CompressedImage, image_callback)
    rospy.spin()
