// Flight.cpp: source code for handling flight planning

#include <string>
#include <cstddef>
#include <iostream>

#include "Flight.h"
#include "Parsing.h"
#include "SetpointScheme.h"

namespace fly_mission
{

// constructor
Flight::Flight(void)
{
	// create our setpoint scheme to store and iterate through the setpoints
	this->setpoint_scheme = PositionTargetScheme();
}

// constructor
Flight::Flight(std::string filename)
{
	// create our setpoint scheme to store and iterate through the setpoints
	this->setpoint_scheme = PositionTargetScheme();

	// read in the flight
	loadFlight(filename, this->setpoint_scheme);
}

// parse out a flight from a file
bool Flight::loadFlight(std::string filename, PositionTargetScheme& pts) {
	bool ret_val = Parsing::flightFromCSV(filename, pts);
	if (!ret_val)
	{
		std::cerr << "flightFromCSV failed." std::endl;
	}
	return ret_val
}
}
