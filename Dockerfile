FROM dustynv/jetson-inference:r32.4.4

# install ros melodic
RUN apt-get update && apt-get install -y lsb-release gnupg curl apt-utils && apt-get clean all 
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt update && apt install -y ros-melodic-ros-base
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
SHELL ["/bin/bash", "-c", "source ~/.bashrc"]

# enable python3 support
RUN apt-get install -y python3-pip python3-yaml
RUN pip3 install rospkg catkin_pkg

# fix cv_bridge
RUN apt-get install -y python-catkin-tools python3-dev python3-numpy
RUN mkdir ~/catkin_build_ws && cd ~/catkin_build_ws
RUN catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
RUN catkin config --install

RUN mkdir src && cd src && git clone -b melodic https://github.com/ros-perception/vision_opencv.git
RUN cd ~/catkin_build_ws 
RUN catkin build cv_bridge 
SHELL ["/bin/bash", "-c", "source install/setup.bash --extend"]

WORKDIR src/

CMD [ "/bin/bash" ]
