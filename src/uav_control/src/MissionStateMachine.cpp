// this is the module responsible for keeping agent sm for collision avoidance
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

#include "StateMachine.h"
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

MissionStateMachine::MissionStateMachine() {
	// initialize the necessary components for our state machine
	
	this->flags.state_entry = true;
	this->flags.state_exit = false;

}

void MissionStateMachine::cycle(void)
{

	// prime the next state to be the same as the current state unless changed
	MissionStates next_state = this->getCurrentState();

	// check for events
	MissionEvents event = MissionEvents::NO_EVENT;//event_ptr->getValue();
	Event * latest_event_ptr = this->checkEvents();
	if (latest_event_ptr != nullptr)
	{
		cout << "there is an event to ingest" << endl;
		MissionEvent* event_ptr = (MissionEvent *) latest_event_ptr;
		event = event_ptr->getValue();
	}

	// primary switch statement for the state machine
	switch (this->getCurrentState()) {

		case MissionStates::INIT:
		
		if (this->flags.state_entry == true) {
			// state entry execution
			debugOut("	In state 'INIT'...", BLUE);
			this->flags.state_entry = false;
		}

		// STATE TRANSFER CONDITIONS 
		// only cycle through the init state once

		if (true) {
			debugOut("	Next state will be 'CHECKING_PREARM'...", CYAN);
			next_state = MissionStates::CHECKING_PREARM;
			this->flags.state_exit = true;
		}

		if (this->flags.state_exit == true) {
			debugOut("	Exiting 'INIT'...", MAGENTA);
			cout << endl;
			this->flags.state_exit = false;
			this->flags.state_entry = true;
		}

		break;


		case MissionStates::CHECKING_PREARM:
		
		if (this->flags.state_entry == true) {
			// state entry execution
			debugOut("	In state 'CHECKING_PREARM'...", BLUE);
			this->flags.state_entry = false;
		}

		// STATE TRANSFER CONDITIONS 
		// here, we wait until the system is done checking that everything which
		// should be done prior to arming has been completed

		if (event == MissionEvents::CHECKS_PREARM_COMPLETE) {
			debugOut("	Prearming checks have completed.", YELLOW);
			debugOut("	Next state will be 'ARMING'...", CYAN);
			next_state = MissionStates::ARMING;
			this->flags.state_exit = true;
		}

		if (this->flags.state_exit == true) {
			debugOut("	Exiting 'CHECKING_PREARM'...", MAGENTA);
			cout << endl;
			this->flags.state_exit = false;
			this->flags.state_entry = true;
		}

		break;
		/*

		case MissionStates::ARMING:
		
		if (this->flags.state_entry == true) {
			// state entry execution
			debugOut("	In state 'ARMING'...", BLUE);
			this->flags.state_entry = false;
		}

		// STATE TRANSFER CONDITIONS 
		// when the system finishes arming, transition into takeoff
		if (event == MissionEvents::AIRCRAFT_ARMED) {
			debugOut("	Aricraft is now armed.", YELLOW);
			debugOut("	Next state will be 'TAKING_OFF'...", CYAN);
			next_state = MissionStates::TAKING_OFF;
			this->flags.state_exit = true;
		}


		if (this->flags.state_exit == true) {
			debugOut("	Exiting 'ARMING'...", MAGENTA);
			cout << endl;
			this->flags.state_exit = false;
			this->flags.state_entry = true;
		}

		break;

		case MissionStates::TAKING_OFF:
		
		if (this->flags.state_entry == true) {
			// state entry execution
			debugOut("	In state 'TAKING_OFF'...", BLUE);
			this->flags.state_entry = false;
		}

		// STATE TRANSFER CONDITIONS 
		// once takeoff is finished, we can run the mission
		if (event == MissionEvents::TAKEOFF_COMPLETE) {
			debugOut("	Takeoff altitude has been reached -- takeoff is now complete.", YELLOW);
			debugOut("	Next state will be 'IN_OFFBOARD'...", CYAN);
			next_state = MissionStates::IN_OFFBOARD;
			this->flags.state_exit = true;
		}


		if (this->flags.state_exit == true) {
			debugOut("	Exiting 'TAKING_OFF'...", MAGENTA);
			cout << endl;
			this->flags.state_exit = false;
			this->flags.state_entry = true;
		}

		break;

		case MissionStates::IN_OFFBOARD:
		
		if (this->flags.state_entry == true) {
			// state entry execution
			debugOut("	In state 'IN_OFFBOARD'...", BLUE);
			this->flags.state_entry = false;
		}

		// SUB-SM

		// STATE TRANSFER CONDITIONS 
		// once takeoff is finished, we can run the mission
		if (event == MissionEvents::OFFBOARD_MISSION_COMPLETE) {
			debugOut("	Mission completed.", YELLOW);
			debugOut("	Next state will be 'RETURNING_TO_HOME'...", CYAN);
			next_state = MissionStates::RETURNING_TO_HOME;
			this->flags.state_exit = true;
		}

		if (this->flags.state_exit == true) {
			debugOut("	Exiting 'TAKING_OFF'...", MAGENTA);
			cout << endl;
			this->flags.state_exit = false;
			this->flags.state_entry = true;
		}

		break;

		case MissionStates::RETURNING_TO_HOME:
		
		if (this->flags.state_entry == true) {
			// state entry execution
			debugOut("	In state 'RETURNING_TO_HOME'...", BLUE);
			this->flags.state_entry = false;
		}

		// STATE TRANSFER CONDITIONS 
		// once takeoff is finished, we can run the mission
		if (event == MissionEvents::REACHED_HOME_COORDS) {
			debugOut("	Got pack to the lat/lon for home.", YELLOW);
			debugOut("	Next state will be 'LANDING'...", CYAN);
			next_state = MissionStates::LANDING;
			this->flags.state_exit = true;
		}

		if (this->flags.state_exit == true) {
			debugOut("	Exiting 'RETURNING_TO_HOME'...", MAGENTA);
			cout << endl;
			this->flags.state_exit = false;
			this->flags.state_entry = true;
		}

		break;

		case MissionStates::LANDING:
		
		if (this->flags.state_entry == true) {
			// state entry execution
			debugOut("	In state 'LANDING'...", BLUE);
			this->flags.state_entry = false;
		}

		// STATE TRANSFER CONDITIONS 
		// once takeoff is finished, we can run the mission
		if (event == MissionEvents::TOUCHED_DOWN) {
			debugOut("	We've landed.", YELLOW);
			debugOut("	Next state will be 'DISARMING'...", CYAN);
			next_state = MissionStates::DISARMING;
			this->flags.state_exit = true;
		}

		if (this->flags.state_exit == true) {
			debugOut("	Exiting 'LANDING'...", MAGENTA);
			cout << endl;
			this->flags.state_exit = false;
			this->flags.state_entry = true;
		}

		break;

		case MissionStates::DISARMING:
		
		if (this->flags.state_entry == true) {
			// state entry execution
			debugOut("	In state 'DISARMING'...", BLUE);
			this->flags.state_entry = false;
		}

		// STATE TRANSFER CONDITIONS 
		// once takeoff is finished, we can run the mission
		if (event == MissionEvents::DISARMED) {
			debugOut("	We've disarmed.", YELLOW);
			debugOut("	Next state will be 'EXIT'...", CYAN);
			next_state = MissionStates::EXIT;
			this->flags.state_exit = true;
		}

		if (this->flags.state_exit == true) {
			debugOut("	Exiting 'DISARMING'...", MAGENTA);
			cout << endl;
			this->flags.state_exit = false;
			this->flags.state_entry = true;
		}

		break;


		case MissionStates::EXIT:
		
		if (this->flags.state_entry == true) {
			// state entry execution
			debugOut("	In state 'EXIT'...", BLUE);
			this->flags.state_entry = false;
		}

		break;
		*/

		default:
		
		debugOut("ERROR: dropped into default state -- state machine has lost state!", RED);
		cout << endl;

		break;
	}

	this->setCurrentState( next_state );
}

