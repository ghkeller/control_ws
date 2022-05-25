// Flight.h: header file for the flight setup functionality

#pragma once

#include <string>
#include <cstddef>
#include <iostream>

#include "SetpointScheme.h"
#include "MissionStateMachine.h"
#include "mavros_msgs/PositionTarget.h"

class Flight
{
	public:
	// constructor without mission loading
	Flight(void);
	// constructor with mission loading
	Flight(std::string);
	
	private:
	// attrs
	PositionTargetScheme setpoint_scheme; // eventually abstract into SetpointScheme to have generality
	MissionStateMachine state_machine;

	// funcs
	bool load(std::string);
	bool execute(void); // possibly change both of these bools to ints with assigned error vals
	bool terminate(void); 
	bool pause(bool); //true: pause or remain paused -- false: unpause or remain unpaused 
};
