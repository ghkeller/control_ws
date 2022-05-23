// FlightParsing.h: header file for parsing out the flight from a delimited file

#pragma once

#include <string>
#include <cstddef>
#include <iostream>
#include <mavros/PositionTarget.h>

#include "Flight.h"
#include "FlightParsing.h"

class FlightParsing
{
	public:
		static flightFromCsv(std::string _path_to_CSV, Flight& _flight);
	
	private:
}
