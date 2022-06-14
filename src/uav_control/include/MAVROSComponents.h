#pragma once

/* system includes */
#include <iostream>
#include <string>
#include <cstddef>
#include <queue>
#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/String.h>
#include <mavros_msgs/SetMode.h>


/* local includes */
using namespace std;

class MAVROSComponents
{
	public:
	MAVROSComponents();
	virtual ~MAVROSComponents() = default;

	//setters
	void setCurrentVehicleState(mavros_msgs::State _current_state)
		{ this->current_vehicle_state = _current_state; };

	void setCurrentVehiclePose( geometry_msgs::PoseStamped _current_pose)
		{ this->current_vehicle_pose = _current_pose; };

		// pseudo-enum declaration
	void setCurrentVehicleTarget( mavros_msgs::PositionTarget _current_target)
		{ this->current_vehicle_target = _current_target; };

	void setWaypointDistanceThresh(float _wp_dist_thresh)
		{ this->waypoint_distance_hit_thresh = _wp_dist_thresh; };

	void setWaypointWaitDuration (float _wp_wait_duration)
		{ this->waypoint_hit_wait_time = _wp_wait_duration; };

	void setCollisionAvoidanceFlag(bool _coll_av_flag)
		{ this->coll_av_flag = _coll_av_flag; }


	//getters
	mavros_msgs::State setCurrentVehicleState()
		{ return this->current_vehicle_state; };

	 geometry_msgs::PoseStamped setCurrentVehiclePose()
		{ return this->current_vehicle_pose; };

	 mavros_msgs::PositionTarget setCurrentVehicleTarget()
		{ return this->current_vehicle_target; };

	float setWaypointDistanceThresh()
		{ return this->waypoint_distance_hit_thresh; };

	float setWaypointWaitDuration()
		{ return this->waypoint_hit_wait_time; };

	bool setCollisionAvoidanceFlag()
		{ return this->coll_av_flag; };

	string getFlightFilename()
		{ return this->flight_name; };

	//pseudo-getters
	void fetchParams();
	
	//pseudo-setters
	void setupServices();
	void setupSubscribers();
	void setupPublishers();

	//callbacks
	//void gcs_alert_cb(const std_msgs::String::ConstPtr&);
	void target_cb(const mavros_msgs::PositionTarget::ConstPtr&);
	void position_cb(const geometry_msgs::PoseStamped::ConstPtr&);
	void state_cb(const mavros_msgs::State::ConstPtr&);

	protected:

	// node handle
	ros::NodeHandle _nh;

	// publishers
	ros::Publisher target_pos_pub;

	// subscribers
	ros::Subscriber state_sub;
	ros::Subscriber position_sub;
	ros::Subscriber target_sub;
	ros::Subscriber gcs_alert_sub;

	// service clients
	ros::ServiceClient arming_client;
	ros::ServiceClient set_mode_client;
	ros::ServiceClient takeoff_client;

	// mission details
	std::string flight_name;
	double waypoint_distance_hit_thresh;
	double waypoint_hit_wait_time;
	int starting_waypoint_number;

	// vehicle information
	mavros_msgs::State current_vehicle_state;
	geometry_msgs::PoseStamped current_vehicle_pose;
	mavros_msgs::PositionTarget current_vehicle_target;

	// option bools
	bool avoidance;

	// sm flags
	bool coll_av_flag;

	//string vars
	std::string state_topic;
	std::string position_topic;
	std::string setpoint_topic; 
	std::string target_topic; 
	std::string gcs_alert_topic; 

	std::string arming_service; 
	std::string set_mode_service;
	std::string takeoff_service;
};

