#pragma once

/* system includes */
#include <iostream>
#include <string>
#include <cstddef>
#include <queue>

/* local includes */

using namespace std;

class ROSMissionComponents
{
	public:
	ROSMissionComponents();
	//setters
	void setCurrentState(mavros_msgs::State _current_state)
		{ this->current_state = _current_state };

	void setCurrentPose( geometry_msgs::PoseStamped _current_pose)
		{ this->current_pose = _current_pose };

	void setCurrentTarget( mavros_msgs::PositionTarget _current_target)
		{ this->current_target = _current_target };

	void setWaypointDistanceThresh(float _wp_dist_thresh)
		{ this->wp_dist_thresh = _wp_dist_thresh };

	void setWaypointWaitDuration (float _wp_wait_duration)
		{ this->wp_wait_duration = _wp_wait_duration };

	void setLastRequest(ros::Time _last_request)
		{ this->last_request = _last_request }

	void setCollisionAvoidanceFlag(bool _coll_av_flag)
		{ this->coll_av_flag = _coll_av_flag }


	//getters
	mavros_msgs::State setCurrentState()
		{ return this->current_state };

	 geometry_msgs::PoseStamped setCurrentPose()
		{ return this->current_pose };

	 mavros_msgs::PositionTarget setCurrentTarget()
		{ return this->current_target };

	float setWaypointDistanceThresh()
		{ return this->wp_dist_thresh };

	float setWaypointWaitDuration()
		{ return this->wp_wait_duration };

	ros::Time setLastRequest()
		{ return this->last_request };

	bool setCollisionAvoidanceFlag()
		{ return this->coll_av_flag };


	//pseudo-getters
	void fetchParams();
	
	//pseudo-setters
	void setupServices();
	void setupSubscribers();
	void setupPublishers();

	//callbacks
	void gcs_alert_cb(const std_msgs::String::ConstPtr&);
	void target_cb(const mavros_msgs::PositionTarget::ConstPtr&);
	void position_cb(const geometry_msgs::PoseStamped::ConstPtr&);
	void state_cb(const mavros_msgs::State::ConstPtr&);

	private:
	// publishers
	ros::Publisher target_pos_pub;

	// subscribers
	ros::Subscriber state_sub;
	ros::Subscriber position_sub;
	ros::Subscriber target_sub;
	ros::Subscriber gcs_alert_sub;

	// service clients
	ros::ServiceClient arming_cl;
	ros::ServiceClient set_mode_cl;
	ros::ServiceClient takeoff_cl;

	// mission details
	std::string flight_name;
	double waypoint_distance_hit_thresh;
	double waypoint_hit_wait_time;
	uint8_t starting_waypoint_number;

	// option bools
	bool avoidance;

	// sm flags
	bool coll_av_flag;

	//string vars
	std::string state_topic;
	std::string position_topic;
	std::string setpoint_topic; 
	std::string gcs_alert_topic; 

	std::string arming_service; 
	std::string set_mode_service;
	std::string takeoff_service;
}

