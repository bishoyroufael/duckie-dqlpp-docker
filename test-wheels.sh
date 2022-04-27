#!/bin/bash

# Wheels should run for 2 seconds and stop
rostopic pub /bishoybot/wheels_driver_node/wheels_cmd duckietown_msgs/WheelsCmdStamped auto 0.5 0.5 &
sleep 2
rostopic pub /bishoybot/wheels_driver_node/wheels_cmd duckietown_msgs/WheelsCmdStamped auto 0.0 0.0  &
sleep 2
pkill -9 rostopic
