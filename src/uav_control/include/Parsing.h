// FlightParsing.h: header file for parsing out the flight from a delimited file

#pragma once

#include <string>
#include <cstddef>
#include <iostream>

#include "SetpointScheme.h"

class Parsing
{
	public:
		static bool flightFromCsv(std::string, PositionTargetScheme&);
	private:
};
