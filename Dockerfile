FROM dustynv/jetson-inference:r32.4.4

# install ros melodic
RUN apt-get update && apt-get install -y --no-install-recommends lsb-release gnupg curl apt-utils

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt update && apt install -y ros-melodic-ros-base python3-pip python3-yaml python3-empy ros-melodic-tf-conversions
RUN pip3 install rospkg catkin_pkg
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
SHELL ["/bin/bash", "-c", "source ~/.bashrc"]

WORKDIR /
RUN git clone https://github.com/duckietown/dt-ros-commons.git
RUN mkdir -p ~/catkin_ws/src

CMD [ "/bin/bash" ]
