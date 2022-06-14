#include <iostream>
#include <string>
#include <queue>
#include <vector>
#include <ros/ros.h>

#include "StateMachine.h"
#include "Flight.h"
#include "WaypointFlight.h"

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

void WaypointFlight::cycle()
{
	// set up next state as current in case nothing changes
	int current_state = this->getCurrentState();
	int next_state = current_state;


	// check for events to ingest
	Event* event_ptr = (Event*) this->checkEvents();
	int event;
	if (event_ptr == nullptr)
		event = Event::NO_EVENT;
	else
		event = event_ptr->value();

	switch (current_state) {

		case WaypointFlight::State::INIT:
		
		if (this->flags.state_entry == true) {
			// state entry execution
			debugOut("	In state 'INIT'...", BLUE);
			this->flags.state_entry = false;
		}

		// STATE TRANSFER CONDITIONS 
		// only cycle through the init state once

		if (true) {
			debugOut("	Next state will be 'CHECKING_PREARM'...", CYAN);
			next_state = WaypointFlight::State::CHECKING_PREARM;
			this->flags.state_exit = true;
		}

		if (this->flags.state_exit == true) {
			debugOut("	Exiting 'INIT'...", MAGENTA);
			cout << endl;
			this->flags.state_exit = false;
			this->flags.state_entry = true;
		}

		break;


		case WaypointFlight::State::CHECKING_PREARM:
		
		if (this->flags.state_entry == true) {
			// state entry execution
			debugOut("	In state 'CHECKING_PREARM'...", BLUE);
			this->flags.state_entry = false;
		}

		// STATE TRANSFER CONDITIONS 
		// here, we wait until the system is done checking that everything which
		// should be done prior to arming has been completed

		if (event == WaypointFlight::Event::CHECKS_PREARM_COMPLETE) {
			debugOut("	Prearming checks have completed.", YELLOW);
			debugOut("	Next state will be 'ARMING'...", CYAN);
			next_state = WaypointFlight::State::ARMING;
			this->flags.state_exit = true;
		}

		if (this->current_vehicle_state.connected)
		{
			debugOut("	The vehicle fcu is connected via ROS.", YELLOW);
			queue<StateMachine::Event *> events( { new WaypointFlight::Event( WaypointFlight::Event::CHECKS_PREARM_COMPLETE ) } );
			this->registerEvents( events );
		}
			

		if (this->flags.state_exit == true) {
			debugOut("	Exiting 'CHECKING_PREARM'...", MAGENTA);
			cout << endl;
			this->flags.state_exit = false;
			this->flags.state_entry = true;
		}

		break;

		/*

		case WaypointFlight::State::ARMING:
		
		if (this->flags.state_entry == true) {
			// state entry execution
			debugOut("	In state 'ARMING'...", BLUE);
			this->flags.state_entry = false;
		}

		// STATE TRANSFER CONDITIONS 
		// when the system finishes arming, transition into takeoff
		if (event == WaypointFlight::Event::AIRCRAFT_ARMED) {
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
		case WaypointFlight::State::INIT:
		
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
		case WaypointFlight::State::INIT:
		
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
		case WaypointFlight::State::INIT:
		
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
		case WaypointFlight::State::INIT:
		
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
		case WaypointFlight::State::INIT:
		
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
		case WaypointFlight::State::INIT:
		
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


	// make state change here
	this->setCurrentState( next_state );
}

