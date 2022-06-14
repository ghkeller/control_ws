// Flight.cpp: source code for handling flight planning

#include <string>
#include <cstddef>
#include <iostream>
#include <queue>

#include "Flight.h"
#include "Parsing.h"
#include "SetpointScheme.h"
#include "StateMachine.h"

using namespace std;

// constructor
Flight::Flight(void) : StateMachine {}, MAVROSComponents {} 
{
	// create our setpoint scheme to store and iterate through the setpoints
	this->setpoint_scheme = PositionTargetScheme();
}

// constructor
Flight::Flight(std::string filename) : StateMachine {}, MAVROSComponents {} 
{
	// create our setpoint scheme to store and iterate through the setpoints
	this->setpoint_scheme = PositionTargetScheme();

	// read in the flight
	load(filename);
}

// parse out a flight from a file
bool Flight::load(std::string filename) {
	// setpoint scheme must be instantiated before we attempt to load a flight
	bool ret_val = Parsing::flightFromCsv(filename, this->setpoint_scheme);
	if (!ret_val)
	{
		std::cerr << "flightFromCSV failed." << std::endl;
	}
	return ret_val;
}


// parse out a flight from a file
bool Flight::load() {
	// setpoint scheme must be instantiated before we attempt to load a flight
	bool ret_val = Parsing::flightFromCsv(this->flight_name, this->setpoint_scheme);
	if (!ret_val)
	{
		std::cerr << "flightFromCSV failed." << std::endl;
	}
	return ret_val;
}

bool Flight::start()
{
	return true;
}

bool Flight::stop()
{
	return true;
}

bool Flight::pause()
{
	return true;
}

bool Flight::unpause()
{
	return true;
}
