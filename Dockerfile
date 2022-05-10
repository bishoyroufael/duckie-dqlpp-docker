FROM dustynv/jetson-inference:r32.4.4

# install ros melodic
RUN apt-get update &&\
	apt-get install -y --no-install-recommends lsb-release gnupg curl apt-utils &&\
	sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' &&\
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - &&\
	apt update &&\
	apt install -y net-tools vim ros-melodic-ros-base python3-pip python3-yaml python3-empy ros-melodic-tf-conversions &&\
	pip3 install rospkg catkin_pkg &&\
	echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc


RUN mkdir -p /root/catkin_ws/src &&\
	git clone https://github.com/duckietown/dt-ros-commons.git &&\
	cp -r dt-ros-commons/packages/* /root/catkin_ws/src/ &&\
	rm -rf dt-ros-commons 

RUN . /opt/ros/melodic/setup.sh &&\
	cd /root/catkin_ws &&\
    catkin_make

RUN mkdir -p /jetson-inference/data/networks &&\
	cd /jetson-inference/data/networks &&\
	curl https://nvidia.box.com/shared/static/frgbiqeieaja0o8b0eyb87fjbsqd4zup.gz -L -o "a.gz" &&\
	curl https://nvidia.box.com/shared/static/ai2sxrp1tg8mk4j0jbrw3vthqjp8x0af.gz -L -o "b.gz" &&\
	curl https://nvidia.box.com/shared/static/3umpq9yrv3nj3ltiwlooijx5of414gbh.gz -L -o "c.gz" &&\
	tar -zxvf a.gz &&\
	tar -zxvf b.gz &&\
	tar -zxvf c.gz &&\
	rm -rf *.gz

RUN python -m pip install --upgrade pip

RUN apt-get -y install pkg-config libhdf5-100 libhdf5-dev && pip install h5py==2.10.0

RUN apt-get -y install llvm-10* && ln -s /usr/bin/llvm-config-10 /usr/bin/llvm-config

RUN pip install flask easydict tianshou

RUN apt-get -y install python3-sklearn libaec-dev libblosc-dev libffi-dev libbrotli-dev libboost-all-dev libbz2-dev &&\
	apt-get -y install libgif-dev libopenjp2-7-dev liblcms2-dev libjpeg-dev libjxr-dev liblz4-dev liblzma-dev libpng-dev libsnappy-dev libwebp-dev libzopfli-dev libzstd-dev

ENV LC_CTYPE en_US.UTF-8
ENV LANG en_US.UTF-8

RUN pip3 install scikit-image

RUN apt-get install psmisc && pip install -U pyopenssl 


WORKDIR /root/catkin_ws/
COPY test-wheels.sh .
# ADD src .

RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/bin/bash"]
