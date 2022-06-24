// MAVROSComponents.cpp: source code for standard ROS components relevant to offboard ctrl

/* system includes */
#include <iostream>
#include <string>
#include <cstddef>
#include <ros/ros.h>
#include <ros/package.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>

/* local includes */
#include "MAVROSComponents.h"

using namespace std;

MAVROSComponents::MAVROSComponents()
{
	// initialize ros components
	this->fetchParams();
	this->setupPublishers();
	this->setupSubscribers();
	this->setupServices();
}

void MAVROSComponents::fetchParams()
{
	//flight/mission parameters
    this->_nh.getParam(ros::this_node::getNamespace() + "/control/flight_fname", this->flight_name);
    this->_nh.getParam(ros::this_node::getNamespace() + "/control/avoidance", this->avoidance);
    this->_nh.getParam(ros::this_node::getNamespace() + "/control/waypoint_distance_hit_thresh", this->waypoint_distance_hit_thresh);
    this->_nh.getParam(ros::this_node::getNamespace() + "/control/waypoint_hit_wait_time", this->waypoint_hit_wait_time);
    this->_nh.getParam(ros::this_node::getNamespace() + "/control/starting_waypoint_number", this->starting_waypoint_number);

	//topics
    this->_nh.getParam(ros::this_node::getNamespace() + "/mavros/state_topic", this->state_topic);
    this->_nh.getParam(ros::this_node::getNamespace() +  "/mavros/position_topic", this->position_topic);
    this->_nh.getParam(ros::this_node::getNamespace() + "/mavros/setpoint_topic", this->setpoint_topic);
    this->_nh.getParam(ros::this_node::getNamespace() + "/mavros/target_topic", this->target_topic);
    this->_nh.getParam(ros::this_node::getNamespace() + "/gcs/alert_topic", this->gcs_alert_topic);

	//services
    this->_nh.getParam(ros::this_node::getNamespace() + "/mavros/arming_service", this->arming_service); 
    this->_nh.getParam(ros::this_node::getNamespace() + "/mavros/mode_service", this->set_mode_service);
    this->_nh.getParam(ros::this_node::getNamespace() + "/mavros/takeoff_service", this->takeoff_service);
}

void MAVROSComponents::setupServices()
{
	
    this->arming_client = this->_nh.serviceClient<mavros_msgs::CommandBool>
            (this->arming_service);
    this->set_mode_client = this->_nh.serviceClient<mavros_msgs::SetMode>
            (this->set_mode_service);
    this->takeoff_client = this->_nh.serviceClient<mavros_msgs::CommandTOL>
    		(this->takeoff_service);
}

void MAVROSComponents::setupSubscribers()
{
    
	ROS_INFO("Setting up the state subscriber.");
    this->state_sub = this->_nh.subscribe<mavros_msgs::State>
            (this->state_topic, 10, &MAVROSComponents::state_cb, this);
	ROS_INFO("Setting up the position subscriber.");
    this->position_sub = this->_nh.subscribe<geometry_msgs::PoseStamped>
            (this->position_topic, 10, &MAVROSComponents::position_cb, this);
	ROS_INFO("Setting up the target subscriber.");
    this->target_sub = this->_nh.subscribe<mavros_msgs::PositionTarget>
            (this->target_topic, 10, &MAVROSComponents::target_cb, this);
    //this->gcs_alert_sub = this->_nh.subscribe<std_msgs::String>
            //(this->gcs_alert_topic, 10, &MAVROSComponents::gcs_alert_cb, this);

}

void MAVROSComponents::setupPublishers(void)
{
    this->target_pos_pub = this->_nh.advertise<mavros_msgs::PositionTarget>
            (this->setpoint_topic, 100);
}


void MAVROSComponents::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    this->current_vehicle_state = *msg;
}

void MAVROSComponents::position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	this->current_vehicle_pose = *msg;
}

void MAVROSComponents::target_cb(const mavros_msgs::PositionTarget::ConstPtr& msg)
{
    this->current_vehicle_target = *msg;
}

/*
void MAVROSComponents::gcs_alert_cb(const std_msgs::String::ConstPtr& msg)
{
	//parse our the message
	std::string recv = msg->data;
    ROS_INFO("Received message from vehicle 0: [%s]", recv.c_str());

	//compare message to cases for flag setting
    if (recv.compare("STOP") == 0) {
    	ROS_INFO("    GCS has commanded STOP.");
    	this->coll_av_flag = true;
    	return;
    }
}
*/
