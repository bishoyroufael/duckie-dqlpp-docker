#!/bin/bash
IP_ADD_BOT=$(ifconfig wlan0 | grep -w "inet" | tr -s " " | cut -f3 -d" ")
PORT=11311
echo BUILDING IMAGE ..
echo GOT IP OF BOT: $IP_ADD_BOT
echo USING PORT: 11311

docker build --net=host -e ROS_MASTER_URI=http://$IP_ADD_BOT:$PORT --no-cache -t dqlpp-ros .
