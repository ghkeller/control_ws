#!/bin/bash

# set up the env
source ~/control_ws/devel/setup.bash

# execute the launch file
roslaunch uav_control uav_offboard_control.launch
