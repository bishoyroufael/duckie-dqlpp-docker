#!/bin/bash
ROBOT_NAME=bishoybot

docker -H $ROBOT_NAME.local  build --network="host" -t dqlpp-ros .
