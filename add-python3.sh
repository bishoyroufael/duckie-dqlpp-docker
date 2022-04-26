#!/bin/bash
sudo apt-get install -y python3-pip python3-yaml
sudo pip3 install rospkg catkin_pkg

# fix cv_bridge
sudo apt-get install -y python-catkin-tools python3-dev python3-numpy
mkdir ~/catkin_build_ws && cd ~/catkin_build_ws
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
catkin config --install

mkdir src && cd src && git clone -b melodic https://github.com/ros-perception/vision_opencv.git
cd ~/catkin_build_ws 
catkin build cv_bridge 
source install/setup.bash --extend
