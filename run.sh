#!/bin/bash

# IP_ADD_BOT=$(ifconfig wlan0 | grep -w "inet" | tr -s " " | cut -f3 -d" ")
PORT=11311
echo RUNNING CONTAINER..
# echo GOT IP OF BOT: $IP_ADD_BOT
echo USING PORT: $PORT
ROBOT_NAME=bishoybot

# docker run --network=host -it -e "ROS_MASTER_URI=http://localhost:$PORT" -e "ROS_HOSTNAME=127.0.0.1" dqlpp-ros:latest 
sudo docker -H $ROBOT_NAME.local run --rm -it --gpus all -v $(pwd)/src:/root/catkin_ws/dqlpp --privileged --net=host  dqlpp-ros:latest 

