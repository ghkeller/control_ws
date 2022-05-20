// ROSMissionComponents.cpp: source code for standard ROS components relevant to offboard ctrl

/* system includes */
#include <iostream>
#include <string>
#include <cstddef>
#include <ros/ros.h>
#include <ros/package.h>

/* local includes */
#include "ROSMissionComponents.h"


using namespace std;

ROSMissionComponents::ROSMissionComponents(ros::NodeHandle _nh)
{
	// initialize ros components
	this->fetchParams(_nh);
	this->setupPublishers(_nh);
	this->setupSubscribers(_nh);
	this->setupServices(_nh);
}

void ROSMissionComponents::fetchParams(ros::NodeHandle _nh)
{
	//flight/mission parameters
    _nh.getParam(ros::this_node::getNamespace() + "/control/flight_fname", this->flight_fname);
    _nh.getParam(ros::this_node::getNamespace() + "/control/avoidance", this->avoidance);
    _nh.getParam(ros::this_node::getNamespace() + "/control/waypoint_distance_hit_thresh", this->waypoint_distance_hit_thresh);
    _nh.getParam(ros::this_node::getNamespace() + "/control/waypoint_hit_wait_time", this->waypoint_hit_wait_time);
    _nh.getParam(ros::this_node::getNamespace() + "/control/starting_waypoint_number", this->starting_waypoint_number);

	//topics
    _nh.getParam(ros::this_node::getNamespace() + "/mavros/state_topic", this->state_topic);
    _nh.getParam(ros::this_node::getNamespace() +  "/mavros/pose_topic", this->pose_topic);
    _nh.getParam(ros::this_node::getNamespace() + "/mavros/setpoint_topic", this->setpoint_topic);
    _nh.getParam(ros::this_node::getNamespace() + "/gcs/alert_topic", this->gcs_alert_topic);

	//services
    _nh.getParam(ros::this_node::getNamespace() + "/mavros/arming_service", this->arming_service); 
    _nh.getParam(ros::this_node::getNamespace() + "/mavros/mode_service", this->set_mode_service);
    _nh.getParam(ros::this_node::getNamespace() + "/mavros/takeoff_service", this->takeoff_service);

}

void ROSMissionComponents::setupServices(ros::NodeHandle _nh)
{
    this->arming_client = _nh.serviceClient<mavros_msgs::CommandBool>
            (this->arming_service);
    this->set_mode_client = _nh.serviceClient<mavros_msgs::SetMode>
            (this->mode_service);
    this->takeoff_cl = _nh.serviceClient<mavros_msgs::CommandTOL>
    		(this->takeoff_service);
}

void ROSMissionComponents::setupSubscribers(ros::NodeHandle _nh)
{
    
	//subs, pubs, etc
    ros::Subscriber state_sub = _nh.subscribe<mavros_msgs::State>
            (this->state_topic, 10, this->state_cb);
    ros::Subscriber position_sub = _nh.subscribe<geometry_msgs::PoseStamped>
            (this->pose_topic, 10, this->pose_cb);
    ros::Subscriber target_sub = _nh.subscribe<geometry_msgs::PoseStamped>
            (this->target_topic, 10, this->target_cb);
    ros::Subscriber gcs_alert_sub = _nh.subscribe<std_msgs::String>
            (this->gcs_alert_topic, 10, this->gcs_alert_cb);

}

void ROSMissionComponents::setupPublishers(ros::NodeHandle _nh)
{
    target_pos_pub = _nh.advertise<mavros_msgs::PositionTarget>
            (this->setpoint_topic, 100);
                //declare subscribers
}


void ROSMissionComponents::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    this->current_state = *msg;
}

void ROSMissionComponents::position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	this->current_pose = *msg;
}

void ROSMissionComponents::target_cb(const mavros_msgs::PositionTarget::ConstPtr& msg)
{
    this->current_target = *msg;
}

void ROSMissionComponents::gcs_alert_cb(const std_msgs::String::ConstPtr& msg)
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
