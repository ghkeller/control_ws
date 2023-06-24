#!/bin/bash

# source the env
source $HOME/control_ws/devel/setup.bash

# run the ros node
rosrun uav_control go_to_point_and_start_terr_foll 
