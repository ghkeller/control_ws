// Flight.h: header file for the flight setup functionality

#pragma once

#include <string>
#include <cstddef>
#include <iostream>

#include "SetpointScheme.h"
#include "Parsing.h"
#include "mavros_msgs/PositionTarget.h"

class Flight
{
	public:
	Flight(void);
	Flight(std::string);
	
	private:
	PositionTargetScheme setpoint_scheme; // eventually abstract into SetpointScheme to have generality
	bool loadFlight(std::string);
};
