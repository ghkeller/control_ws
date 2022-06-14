// Flight.h: header file for the flight setup functionality

#pragma once

#include <string>
#include <cstddef>
#include <iostream>

#include "SetpointScheme.h"
#include "MAVROSComponents.h"
#include "StateMachine.h"

class Flight: public StateMachine, public MAVROSComponents
{
	public:
	// constructor without mission loading
	Flight(void);

	// constructor with mission loading
	Flight(std::string);
	virtual ~Flight() = default;

	// funcs
	bool load();
	bool load(std::string);
	bool start(void); // possibly change both of these bools to ints with assigned error vals
	bool stop(void); 
	bool pause(void); //true: pause or remain paused -- false: unpause or remain unpaused 
	bool unpause(void); //true: pause or remain paused -- false: unpause or remain unpaused 
	
	protected:
	// attrs
	PositionTargetScheme setpoint_scheme; // eventually abstract into SetpointScheme to have generality
};
