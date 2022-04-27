FROM dustynv/jetson-inference:r32.4.4

# install ros melodic
RUN apt-get update &&\
	apt-get install -y --no-install-recommends lsb-release gnupg curl apt-utils &&\
	sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' &&\
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - &&\
	apt update &&\
	apt install -y ros-melodic-ros-base python3-pip python3-yaml python3-empy ros-melodic-tf-conversions &&\
	pip3 install rospkg catkin_pkg &&\
	echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc


RUN mkdir -p /root/catkin_ws/src &&\
	git clone https://github.com/duckietown/dt-ros-commons.git &&\
	cp -r dt-ros-commons/packages/* /root/catkin_ws/src/ &&\
	rm -rf dt-ros-commons 

RUN . /opt/ros/melodic/setup.sh &&\
	cd /root/catkin_ws &&\
    catkin_make



WORKDIR /root/catkin_ws/
COPY test-wheels.sh .

RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/bin/bash"]
