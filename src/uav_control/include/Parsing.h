// FlightParsing.h: header file for parsing out the flight from a delimited file

#pragma once

#include <string>
#include <cstddef>
#include <iostream>

#include "Flight.h"
#include "mavros_msgs/PositionTarget.h"

class FlightParsing
{
	public:
		static void flightFromCsv(std::string _path_to_CSV, Flight& _flight);
	
	private:
};
