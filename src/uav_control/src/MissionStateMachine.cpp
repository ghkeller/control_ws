// this is the module responsible for keeping agent sm for collision avoidance
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

#include "MissionStateMachine.h"

#define DEBUG

#define BLACK 30
#define RED 31
#define GREEN 32
#define YELLOW 33
#define BLUE 34
#define MAGENTA 35
#define CYAN 36
#define WHITE 37

using namespace std;

void debugOut(string str, int color_code)
{
#ifdef DEBUG
	cout << "\033[1;" << color_code << "m" << str << "\033[0m\n";
#endif
}

// constructor
MissionStateMachine::MissionStateMachine() {
	// initialize the necessary components for our state machine
	this->current_state = State::INIT;
	this->flags.state_entry = true;
	this->flags.state_exit = false;

	// state name string access
	this->state_map[State::INIT] = "INIT";
	this->state_map[State::CHECKING_PREARM] = "CHECKING_PREARM";
	this->state_map[State::ARMING] = "ARMING";
}


MissionStateMachine::State MissionStateMachine::getCurrentState(void)
{
	return this->current_state;
}


void MissionStateMachine::setCurrentState(MissionStateMachine::State state)
{
	this->current_state = state;
}


MissionStateMachine::Event MissionStateMachine::checkEvents()
{
	// get event from the front of the FIFO buffer (replace with priority buffer in future?)
	Event event;

	// if no events, return NO_EVENT
	if (this->event_queue.empty()) {
		return Event::NO_EVENT;
	} 

	// pop and return the event
	event = this->event_queue.front();
	this->event_queue.pop();
	return event;
}

void MissionStateMachine::registerEvent(MissionStateMachine::Event event)
{
	// add the event to the queue
	this->event_queue.push(event);
}

void MissionStateMachine::cycle(void)
{

	State next_state = this->current_state;

	// check for events
	Event event = this->checkEvents();

	switch (this->current_state) {

		case State::INIT:
		
		if (this->flags.state_entry == true) {
			// state entry execution
			debugOut("	In state 'INIT'...", BLUE);
			this->flags.state_entry = false;
		}

		// STATE TRANSFER CONDITIONS 
		// only cycle through the init state once

		if (true) {
			debugOut("	Next state will be 'CHECKING_PREARM'...", CYAN);
			next_state = State::CHECKING_PREARM;
			this->flags.state_exit = true;
		}

		if (this->flags.state_exit == true) {
			debugOut("	Exiting 'INIT'...", MAGENTA);
			cout << endl;
			this->flags.state_exit = false;
			this->flags.state_entry = true;
		}

		break;

		case State::CHECKING_PREARM:
		
		if (this->flags.state_entry == true) {
			// state entry execution
			debugOut("	In state 'CHECKING_PREARM'...", BLUE);
			this->flags.state_entry = false;
		}

		// STATE TRANSFER CONDITIONS 
		// here, we wait until the system is done checking that everything which
		// should be done prior to arming has been completed

		if (event == Event::CHECKS_PREARM_COMPLETE) {
			debugOut("	Prearming checks have completed.", YELLOW);
			debugOut("	Next state will be 'ARMING'...", CYAN);
			next_state = State::ARMING;
			this->flags.state_exit = true;
		}

		if (this->flags.state_exit == true) {
			debugOut("	Exiting 'CHECKING_PREARM'...", MAGENTA);
			cout << endl;
			this->flags.state_exit = false;
			this->flags.state_entry = true;
		}

		break;

		case State::ARMING:
		
		if (this->flags.state_entry == true) {
			// state entry execution
			debugOut("	In state 'ARMING'...", BLUE);
			this->flags.state_entry = false;
		}

		// STATE TRANSFER CONDITIONS 
		// when the system finishes arming, transition into takeoff
		if (event == Event::AIRCRAFT_ARMED) {
			debugOut("	Aricraft is now armed.", YELLOW);
			debugOut("	Next state will be 'TAKING_OFF'...", CYAN);
			next_state = State::TAKING_OFF;
			this->flags.state_exit = true;
		}


		if (this->flags.state_exit == true) {
			debugOut("	Exiting 'ARMING'...", MAGENTA);
			cout << endl;
			this->flags.state_exit = false;
			this->flags.state_entry = true;
		}

		break;

		case State::TAKING_OFF:
		
		if (this->flags.state_entry == true) {
			// state entry execution
			debugOut("	In state 'TAKING_OFF'...", BLUE);
			this->flags.state_entry = false;
		}

		// STATE TRANSFER CONDITIONS 
		// once takeoff is finished, we can run the mission
		if (event == Event::TAKEOFF_COMPLETE) {
			debugOut("	Takeoff altitude has been reached -- takeoff is now complete.", YELLOW);
			debugOut("	Next state will be 'IN_OFFBOARD'...", CYAN);
			next_state = State::IN_OFFBOARD;
			this->flags.state_exit = true;
		}


		if (this->flags.state_exit == true) {
			debugOut("	Exiting 'TAKING_OFF'...", MAGENTA);
			cout << endl;
			this->flags.state_exit = false;
			this->flags.state_entry = true;
		}

		break;

		case State::IN_OFFBOARD:
		
		if (this->flags.state_entry == true) {
			// state entry execution
			debugOut("	In state 'IN_OFFBOARD'...", BLUE);
			this->flags.state_entry = false;
		}

		// SUB-SM

		// STATE TRANSFER CONDITIONS 
		// once takeoff is finished, we can run the mission
		if (event == Event::OFFBOARD_MISSION_COMPLETE) {
			debugOut("	Mission completed.", YELLOW);
			debugOut("	Next state will be 'RETURNING_TO_HOME'...", CYAN);
			next_state = State::RETURNING_TO_HOME;
			this->flags.state_exit = true;
		}

		if (this->flags.state_exit == true) {
			debugOut("	Exiting 'TAKING_OFF'...", MAGENTA);
			cout << endl;
			this->flags.state_exit = false;
			this->flags.state_entry = true;
		}

		break;

		case State::RETURNING_TO_HOME:
		
		if (this->flags.state_entry == true) {
			// state entry execution
			debugOut("	In state 'RETURNING_TO_HOME'...", BLUE);
			this->flags.state_entry = false;
		}

		// SUB-SM

		// STATE TRANSFER CONDITIONS 
		// once takeoff is finished, we can run the mission
		if (event == Event::REACHED_HOME_COORDS) {
			debugOut("	Got pack to the lat/lon for home.", YELLOW);
			debugOut("	Next state will be 'LANDING'...", CYAN);
			next_state = State::LANDING;
			this->flags.state_exit = true;
		}

		if (this->flags.state_exit == true) {
			debugOut("	Exiting 'RETURNING_TO_HOME'...", MAGENTA);
			cout << endl;
			this->flags.state_exit = false;
			this->flags.state_entry = true;
		}

		break;

		case State::LANDING:
		
		if (this->flags.state_entry == true) {
			// state entry execution
			debugOut("	In state 'LANDING'...", BLUE);
			this->flags.state_entry = false;
		}

		// STATE TRANSFER CONDITIONS 
		// once takeoff is finished, we can run the mission
		if (event == Event::TOUCHED_DOWN) {
			debugOut("	We've landed.", YELLOW);
			debugOut("	Next state will be 'DISARMING'...", CYAN);
			next_state = State::DISARMING;
			this->flags.state_exit = true;
		}

		if (this->flags.state_exit == true) {
			debugOut("	Exiting 'LANDING'...", MAGENTA);
			cout << endl;
			this->flags.state_exit = false;
			this->flags.state_entry = true;
		}

		break;

		case State::DISARMING:
		
		if (this->flags.state_entry == true) {
			// state entry execution
			debugOut("	In state 'DISARMING'...", BLUE);
			this->flags.state_entry = false;
		}

		// STATE TRANSFER CONDITIONS 
		// once takeoff is finished, we can run the mission
		if (event == Event::DISARMED) {
			debugOut("	We've disarmed.", YELLOW);
			debugOut("	Next state will be 'EXIT'...", CYAN);
			next_state = State::EXIT;
			this->flags.state_exit = true;
		}

		if (this->flags.state_exit == true) {
			debugOut("	Exiting 'DISARMING'...", MAGENTA);
			cout << endl;
			this->flags.state_exit = false;
			this->flags.state_entry = true;
		}

		break;


		case State::EXIT:
		
		if (this->flags.state_entry == true) {
			// state entry execution
			debugOut("	In state 'EXIT'...", BLUE);
			this->flags.state_entry = false;
		}

		break;

		default:
		
		debugOut("ERROR: dropped into default state -- state machine has lost state!", RED);
		cout << endl;

		break;
	}

	this->current_state = next_state;
}



/*
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


		// STATE TRANSFER CONDITIONS 
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


		// STATE TRANSFER CONDITIONS 
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

		// STATE TRANSFER CONDITIONS
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


		// STATE TRANSFER CONDITIONS
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


		// STATE TRANSFER CONDITIONS 
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

		// STATE TRANSFER CONDITIONS
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
*/
