// Flight.h: header file for the flight setup functionality

#pragma once

#include <string>
#include <cstddef>
#include <iostream>

#include "FlightParsing.h"
#include "mavros_msgs/PositionTarget.h"

class Flight
{
	public:
	Flight(void);
	Flight(std::string);
	
	private:
	bool loadFlight(std::string);
};
