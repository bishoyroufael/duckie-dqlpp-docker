import rospy
import nvidia
import flask
import numpy as np
import base64
from sensor_msgs.msg import CompressedImage
import cv2
from flask import app, Flask, jsonify, Response, render_template
from collections import deque
from config import cfg
from utils import ApiUtils
from videofeed import VideoFeed


W, H = nvidia.get_depth_field_dims()
# Store image buffers
feed = VideoFeed(W, H)

# Flask initialization
app = Flask(__name__)
app.debug = True


# Flask Endpoints
@app.route('/rgb_video')
def rgb_video():
    return Response(ApiUtils.get_stream_buffer(feed), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/depth_video')
def depth_video():
    return Response(ApiUtils.get_stream_buffer(feed, depth=True), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return render_template('index.html')


# Callback for subscriber
# Training happen here each time an image is recieved
def image_callback(msg):
    # print("Received an image!")
    global feed
    try:
        feed.update(msg) 

    except Exception as e:
        print(e)

if __name__ == '__main__':
    rospy.init_node('deep_q_learning')
    rospy.Subscriber(cfg.camera_topic, CompressedImage, image_callback)
    # rospy.Publisher(cfg.wheel_topic)

    app.run(host='0.0.0.0')
    rospy.spin()