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


	//pseudo-getters
	void getParams()
	
	//pseudo-setters
	void setTopics()
	void setServices()
	void setSubscribers()
	void setPublishers

	private:
	// mission details
	std::string flight_name;
	double waypoint_distance_hit_thresh;
	double waypoint_hit_wait_time;
	uint8_t starting_waypoint_number;

	// option bools
	bool avoidance;

	// sm flags
	bool coll_av_flag

	// use this for timers
	ros::Publisher target_pos_pub;

	//topic vars
	std::string state_topic;
	std::string position_topic;
	std::string setpoint_topic; 

}

