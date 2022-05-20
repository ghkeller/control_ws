// ROSMissionComponents.cpp: source code for standard ROS components relevant to offboard ctrl

/* system includes */
#include <iostream>
#include <string>
#include <cstddef>

/* local includes */
#include "ROSMissionComponents.h"

using namespace std;

ROSMissionComponents::ROSMissionComponents(ros::NodeHandle _nh,
		, ros::Subscriber _state_topic,
		ros::Subscriber _pose_topic, )
{
	// initialize ros components
	this->getParams(_nh);

	// target_pos_bub
	this->target_pos_pub;

	//topic vars
	this->state_topic;
	this->position_topic;
	this->setpoint_topic; 


}

void ROSMissionComponents::getParams(ros::NodeHandle _nh)
{
    _nh.getParam(ros::this_node::getNamespace() + "/control/flight_fname", flight_fname);
    _nh.getParam(ros::this_node::getNamespace() + "/control/avoidance", avoidance);
    _nh.getParam(ros::this_node::getNamespace() + "/control/waypoint_distance_hit_thresh", waypoint_distance_hit_thresh);
    _nh.getParam(ros::this_node::getNamespace() + "/control/waypoint_hit_wait_time", waypoint_hit_wait_time);
    _nh.getParam(ros::this_node::getNamespace() + "/control/starting_waypoint_number", starting_waypoint_number);
}

void ROSMissionComponents::setTopicsAndServices(ros::NodeHandle _nh)
{
	//topics
    = ros::this_node::getNamespace() + "/mavros/state";
    = ros::this_node::getNamespace() +  "/mavros/local_position/pose";
    = ros::this_node::getNamespace() + "/mavthread/setpoint_raw/local";
    
	//services
    std::string arming_service = ros::this_node::getNamespace() + "/mavros/cmd/arming";
    std::string set_mode_service = ros::this_node::getNamespace() + "/mavros/set_mode";
    std::string takeoff_service = ros::this_node::getNamespace() + "/mavros/cmd/takeoff";

	//subs, pubs, etc
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            (state_topic, 10, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            (position_topic, 10, position_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            (arming_service);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            (set_mode_service);
    ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>
    		(takeoff_service);
    target_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
            (setpoint_topic, 100);
                //declare subscribers
    ros::Subscriber gcs_alert_sub = nh.subscribe<std_msgs::String>
            ("/gcs/vehicle_alert", 10, gcs_alert_cb);
}


