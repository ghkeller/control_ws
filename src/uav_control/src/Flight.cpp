// Flight.cpp: source code for handling flight planning

#include <string>
#include <cstddef>
#include <iostream>
#include <queue>

#include "Flight.h"
#include "Parsing.h"
#include "SetpointScheme.h"
#include "MissionStateMachine.h"

using namespace std;

namespace fly_mission
{

// constructor
Flight::Flight(void)
{
	// create a state machine to move through for different parts of the flight
	this->state_machine = MissionStateMachine();

	// create our setpoint scheme to store and iterate through the setpoints
	this->setpoint_scheme = PositionTargetScheme();
}

// constructor
Flight::Flight(std::string filename)
{
	// create our setpoint scheme to store and iterate through the setpoints
	this->setpoint_scheme = PositionTargetScheme();

	// read in the flight
	load(filename);
}

// parse out a flight from a file
bool Flight::load(std::string filename) {
	// setpoint scheme must be instantiated before we attempt to load a flight
	bool ret_val = Parsing::flightFromCSV(filename, this->setpoint_scheme);
	if (!ret_val)
	{
		std::cerr << "flightFromCSV failed." std::endl;
	}
	return ret_val
}

bool Flight::execute(queue<MissionStateMachine::Event>& events_to_register)
{
	// ingest any events which need to be processed
	this->state_machine.registerEvents(events_to_register);

	// cycle the state machine
	this->state_machine.cycle();

	// check state machine flags
	// get the next waypoint
	if (this->state_machine.getNextWaypointFlag() == true) {
		
		if (!this->setpoint_scheme.queueEmpty) {
			this->setpoint_scheme.nextWaypoint();
			this->state_machine.registerEvents(InOffboardStateMachine::Event::NEXT_WAYPOINT_SET);
		} else {
			this->state_machine.registerEvents(MissionStateMachine::Event::ALL_WAYPOINTS_VISITED);
		}
		this->state_machine.setNextWaypointFlag(false);
	}
}



}
