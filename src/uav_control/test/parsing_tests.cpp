// position_target_scheme_test.cpp: test suite for the interfacing with 
// MAVLink APIs

/* system includes */
#include <iostream>
#include <string>
#include <cstddef>

#include "Parsing.h"
#include "SetpointScheme.h"

using namespace std;

namespace parsing_tests
{

int test_1(void) {
    cout << "Test 1: parsing a correct file." << endl;

	string home_str;
	const char* temp = getenv("HOME");
	if (temp == NULL) {
		cerr << "Could not get $HOME env variable value." << endl;
	} else {
		home_str = string(temp);	
	}
	string fname = home_str.append("/control_ws/src/uav_control/flights/good_flight.csv");

	// instantiate a setpoint scheme object to populate with setpoints
	PositionTargetScheme pts = PositionTargetScheme();

	bool ret_val = Parsing::flightFromCsv(fname, pts);
	if (!ret_val) {
		std::cerr << "Error parsing the flight file which should be correct." << std::endl;
		return -1;
	} else {
		std::cout << "Successfully parsed file which should be successfully parsed." << std::endl;
		return 0;
	}
}

int test_2(void) { // TEST FAILS SINCE WE DON'T CHECK FOR NUMBER OF DELIMITED CELLS
    cout << "Test 2: parsing an incorrect file." << endl;

	string home_str;
	const char* temp = getenv("HOME");
	if (temp == NULL) {
		cerr << "Could not get $HOME env variable value." << endl;
	} else {
		home_str = string(temp);	
	}
	string fname = home_str.append("/control_ws/src/uav_control/flights/bad_flight.csv");

	// instantiate a setpoint scheme object to populate with setpoints
	PositionTargetScheme pts = PositionTargetScheme();

	bool ret_val = Parsing::flightFromCsv(fname, pts);
	if (!ret_val) {
		std::cerr << "Error parsing the bad flight file (this is good)." << std::endl;
		return 0;
	} else {
		std::cout << "Successfully parsed file which should be unsuccessfully parsed." << std::endl;
		return -1;
	}
}

int test_3(void) {
    cout << "Test 3: checking the values we got from the file." << endl;

	string home_str;
	const char* temp = getenv("HOME");
	if (temp == NULL) {
		cerr << "Could not get $HOME env variable value." << endl;
	} else {
		home_str = string(temp);	
	}
	string fname = home_str.append("/control_ws/src/uav_control/flights/good_flight.csv");

	// instantiate a setpoint scheme object to populate with setpoints
	PositionTargetScheme pts = PositionTargetScheme();

	bool ret_val = Parsing::flightFromCsv(fname, pts);
	if (!ret_val) {
		std::cerr << "Error parsing the flight file which should be correct." << std::endl;
		return -1;
	} else {
		std::cout << "Successfully parsed file which should be successfully parsed." << std::endl;
	}

	// check the values that we parsed from the file
	mavros_msgs::PositionTarget first_sp = pts.nextSetpoint();
	cout << first_sp.position << endl;

	return 0;
	
}

} //namespace parsing_tests END

using namespace parsing_tests;

int main(void)
{
    int result; // stores the results of the test

    std::cout << "Starting the parssing tests..." << std::endl;

    std::cout << "Test 1:" << std::endl;
    result = test_1();
    if ( result < 0 ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }


    std::cout << "Test 2:" << std::endl;
    result = test_2();
    if ( result < 0 ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }

    std::cout << "Test 3:" << std::endl;
    result = test_3();
    if ( result < 0 ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }

}

