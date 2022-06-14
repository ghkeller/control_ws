/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL

 * 
 * 
 */

//* system includes *//
#include <ros/ros.h>

//* local includes *//
#include "MAVROSComponents.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_mavros_components");
    ROS_INFO("Starting the 'mavros_components' test node.");

	// initialize ros-related parts of this operation
	MAVROSComponents mrc = MAVROSComponents();

	ros::spin();
}
