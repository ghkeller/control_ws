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
#include "SetpointScheme.h"
#include "Timer.h"
#include "WaypointFlight.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_separate_modules");
    ROS_INFO("Starting the 'test_separate_modules' test node.");

	// initialize ros-related parts of this operation
	//MAVROSComponents mrc = MAVROSComponents();

	// instantiate an instance of the Setpoint Scheme
	//PositionTargetScheme pts = PositionTargetScheme();

	// instantiate a timer
	//Timer t = Timer();

	// create a WaypointFlight object
	WaypointFlight wp_flight = WaypointFlight();

	// load the flight referenced in the yaml 
	wp_flight.load();

	ros::spin();
}
