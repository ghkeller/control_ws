// this is the module responsible for keeping agent sm for collision avoidance
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

#include "MissionStateMachine.h"
#include "Timer.h"

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



InOffboardStateMachine::InOffboardStateMachine()
{
	// initialize the necessary components for our state machine
	this->current_state = State::INIT;
	this->flags.state_entry = true;
	this->flags.state_exit = false;

	// state name string access
	this->state_map[State::INIT] = "INIT";
	this->state_map[State::SETTING_TARGET] = "SETTING_TARGET";
	this->state_map[State::CYCLING] = "CYCLING";
	this->state_map[State::STALLING_POST_WP_HIT] = "STALLING_POST_WP_HIT";
}

InOffboardStateMachine::State InOffboardStateMachine::getCurrentState(void)
{
	return this->current_state;
}


void InOffboardStateMachine::setCurrentState(InOffboardStateMachine::State state)
{
	this->current_state = state;
}


InOffboardStateMachine::Event InOffboardStateMachine::checkEvents()
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

void InOffboardStateMachine::registerEvent(InOffboardStateMachine::Event event)
{
	// add the event to the queue
	this->event_queue.push(event);
}


	//for reference:
	//enum class State {INIT, SETTING_TARGET, CYCLING, STALLING_POST_WP_HIT}; 
	//enum class Event {WAYPOINT_HIT, POST_WP_HIT_TIMER_STARTED, POST_WP_HIT_TIMER_FINISHED};

// main body of the state machine
void InOffboardStateMachine::cycle()
{

	// unless a new state is to be traversed to, keep the same state
	State next_state = this->current_state;

	// check for events
	Event event = this->checkEvents();

	//sub state machine for offboard mode
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
			debugOut("	Next state will be 'SETTING_TARGET'...", CYAN);
			next_state = State::SETTING_TARGET;
			this->flags.state_exit = true;
		}

		if (this->flags.state_exit == true) {
			debugOut("	Exiting 'INIT'...", MAGENTA);
			cout << endl;
			this->flags.state_exit = false;
			this->flags.state_entry = true;
		}

		break;

		case State::SETTING_TARGET:

		if (this->flags.state_entry == true) {
			// state entry execution
			debugOut("	In state 'SETTING_TARGET'...", BLUE);
			this->flags.state_entry = false;

			//set the target here -- we'll otherwise be idle in this state

		}

		// we only needed to set the target, so now we can go into cycling
		if (true) {
			debugOut("	Next state will be 'CYCLING'...", CYAN);
			next_state = State::CYCLING;
			this->flags.state_exit = true;
		}

		if (this->flags.state_exit == true) {
			debugOut("	Exiting 'SETTING_TARGET'...", MAGENTA);
			cout << endl;
			this->flags.state_exit = false;
			this->flags.state_entry = true;
		}

		break;

		case State::CYCLING:

		if (this->flags.state_entry == true) {
			// state entry execution
			debugOut("	In state 'CYCLING'...", BLUE);
			this->flags.state_entry = false;
		}

		// we're cycling the setpoint value currently, so now we need to wait unti we
		// hit the waypoint.

		if (event == Event::WAYPOINT_HIT) {
			debugOut("	The waypoint has been hit.", YELLOW);
			debugOut("	Next state will be 'STALLING_POST_WP_HIT'...", CYAN);
			next_state = State::STALLING_POST_WP_HIT;
			this->flags.state_exit = true;
		}

		if (this->flags.state_exit == true) {
			debugOut("	Exiting 'CYCLING'...", MAGENTA);
			cout << endl;
			this->flags.state_exit = false;
			this->flags.state_entry = true;
		}

		break;

		case State::STALLING_POST_WP_HIT:

		if (this->flags.state_entry == true) {
			// state entry execution
			debugOut("	In state 'STALLING_POST_WP_HIT'...", BLUE);
			this->flags.state_entry = false;

			// this is where to start the stall timer
		}

		// we're cycling the setpoint value currently, so now we need to wait unti we
		// hit the waypoint.

		if (false) {
			debugOut("	The waypoint stall timer has now run out.", YELLOW);
			debugOut("	Next state will be 'SETTING_TARGET'...", CYAN);
			next_state = State::STALLING_POST_WP_HIT;
			this->flags.state_exit = true;
		}

		if (this->flags.state_exit == true) {
			debugOut("	Exiting 'STALLING_POST_WP_HIT'...", MAGENTA);
			cout << endl;
			this->flags.state_exit = false;
			this->flags.state_entry = true;
		}

		break;


		default:

		debugOut("ERROR: dropped into default state -- state machine has lost state!", RED);
		cout << endl;

		break;
	}

	this->current_state = next_state;
}
