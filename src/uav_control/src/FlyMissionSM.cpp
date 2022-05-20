// this is the module responsible for keeping agent sm for collision avoidance
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

#include "FlyMissionSM.h"

namespace fly_mission_sm
{

// constructor
FlyMissionSM::FlyMissionSM() {
	// initialize the necessary components for our state machine
	this->current_state = INIT;
	this->flags.state_entry = true;
}


InOffboardSubSM::InOffboardSubSM()
{
	// initialize the necessary components for our state machine
	this->current_state = INIT;
	this->flags.state_entry = true;
}

// main body of the state machine
void InOffboardSubSM::cycle() {
	this::current_state

	//sub state machine for offboard mode
	switch (current_avoid_substate) {
		case INIT:

		if (avoid_substate_entry == true) {
			// state entry execution
			ROS_INFO("In state 'SUB_INIT'...");

			avoid_substate_entry = false;
		}


		/* STATE TRANSFER CONDITIONS */
		// only cycle through the init state once
		if (true) {
			ROS_INFO("Going to state 'SETTING_TARGET'...");
			next_avoid_substate = SETTING_TARGET;
			avoid_substate_entry = true;
		}

		break;

		case SETTING_TARGET:

		if (avoid_substate_entry == true) {
			// state entry execution
			ROS_INFO("In state 'SETTING_TARGET'...");

			if (vec_iterator < pt_vec.size()) {
				// get next waypoint to traverse to
				ROS_INFO("	Getting vector %d", vec_iterator);
				desired_target = pt_vec.at(vec_iterator);

			    ROS_INFO("  position.x: %f", desired_target.pose.position.x);
			    ROS_INFO("  position.y: %f", desired_target.pose.position.y);
			    ROS_INFO("  position.z: %f", desired_target.pose.position.z);

			    ROS_INFO("...redirecting to mavros...");

				vec_iterator++; 
			}
            //start timer
            last_request = ros::Time::now();

			avoid_substate_entry = false;
		}

        target_pos_pub.publish(desired_target);
        ros::spinOnce();


		/* STATE TRANSFER CONDITIONS */
		 //we only want to send the target value once to the handling thread
		 if (true) {
		 	ROS_INFO("Going to state 'CYCLING'...");
		 	next_avoid_substate = CYCLING;
		 	avoid_substate_entry = true;
		 }

		break;

		case CYCLING:

		if (avoid_substate_entry == true) {
			// state entry execution
			ROS_INFO("In state 'CYCLING'...");
            //start timer
            last_request = ros::Time::now();

			avoid_substate_entry = false;
		}

		pt_reached_flag = pt_reached();

		/* STATE TRANSFER CONDITIONS */
		// when we've reached the waypoint, we should transfer to setting or leaving
		if (pt_reached_flag == true && vec_iterator > 3) {
			ROS_INFO("Finished traversing alternate waypoints");
			pt_reached_flag = false;
			return true; // indicating that we are finished traversing
		} else if (pt_reached_flag == true && vec_iterator <= 3) {
			ROS_INFO("Going to state 'SETTING_TARGET'...");
			pt_reached_flag = false;
			next_avoid_substate = SETTING_TARGET;
			avoid_substate_entry = true;
		}

		//if it is taking unreasonably long to get to the next waypoint, assume we are close but toilet bowling
		if (ros::Time::now() - last_request > ros::Duration(wp_visit_timeout)) {
			ROS_INFO("Going to state 'SETTING_TARGET'...");
			next_avoid_substate = SETTING_TARGET;
			avoid_substate_entry = true;
		}

		break;

		default:

		break;
	}

	current_avoid_substate = next_avoid_substate;
	return false; // indicating that we are not finished with our alternate waypoints
}




mavros_msgs::PositionTarget desired_target;

//two vectors which store the position targets and their respective times
std::vector<mavros_msgs::PositionTarget> pt_vec;
std::vector<float> vec_time;

void offboard_sub_sm(void) {
	//sub state machine for offboard mode
	switch (current_offboard_substate) {
		case SUB_INIT:

		if (offboard_substate_entry == true) {
			// state entry execution
			ROS_INFO("In state 'SUB_INIT'...");

			offboard_substate_entry = false;
		}


		/* STATE TRANSFER CONDITIONS */
		// only cycle through the init state once
		if (true) {
			ROS_INFO("Going to state 'SETTING_TARGET'...");
			next_offboard_substate = SETTING_TARGET;
			offboard_substate_entry = true;
		}

		break;

		case SETTING_TARGET:

		if (offboard_substate_entry == true) {
			// state entry execution
			ROS_INFO("In state 'SETTING_TARGET'...");

			if (vec_iterator < pt_vec.size()) {
				// get next waypoint to traverse to
				ROS_INFO("	Getting vector %d", vec_iterator);
				desired_target = pt_vec.at(vec_iterator);

				ROS_INFO("Current target from PT vector: ");
				ROS_INFO("  coordinate frame: %d", desired_target.coordinate_frame);
				ROS_INFO("  type mask: %d", desired_target.type_mask);
				ROS_INFO("  position.x: %f", desired_target.position.x);
				ROS_INFO("  position.y: %f", desired_target.position.y);
				ROS_INFO("  position.z: %f", desired_target.position.z);
				ROS_INFO("  acceleration_or_force.x: %f", desired_target.acceleration_or_force.x);
				ROS_INFO("  acceleration_or_force.y: %f", desired_target.acceleration_or_force.y);
				ROS_INFO("  acceleration_or_force.z: %f", desired_target.acceleration_or_force.z);
				ROS_INFO("  velocity.x: %f", desired_target.velocity.x);
				ROS_INFO("  velocity.y: %f", desired_target.velocity.y);
				ROS_INFO("  velocity.z: %f", desired_target.velocity.z);
				ROS_INFO("  yaw: %f", desired_target.yaw);
				ROS_INFO("  yaw_rate: %f", desired_target.yaw_rate);

				vec_iterator++; 
			}
            //start timer
            last_request = ros::Time::now();

			offboard_substate_entry = false;
		}

        target_pos_pub.publish(desired_target);
        ros::spinOnce();


		/* STATE TRANSFER CONDITIONS */
		 //we only want to send the target value once to the handling thread
		 if (true) {
		 	ROS_INFO("Going to state 'CYCLING'...");
		 	next_offboard_substate = CYCLING;
		 	offboard_substate_entry = true;
		 }

		break;

		case CYCLING:

		if (offboard_substate_entry == true) {
			// state entry execution
			ROS_INFO("In state 'CYCLING'...");
            //start timer
            last_request = ros::Time::now();

			offboard_substate_entry = false;
		}

		/* STATE TRANSFER CONDITIONS */
		// when we've reached the waypoint, we should transfer to waiting
		if (ros::Time::now() - last_request > ros::Duration(vec_time.at(vec_iterator - 1))) {
			ROS_INFO("Going to state 'SETTING_TARGET'...");
			next_offboard_substate = SETTING_TARGET;
			offboard_substate_entry = true;
		}

		break;

		default:

		break;
	}

	current_offboard_substate = next_offboard_substate;
}
