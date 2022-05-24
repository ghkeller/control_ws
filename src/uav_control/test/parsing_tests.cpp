// position_target_scheme_test.cpp: test suite for the interfacing with 
// MAVLink APIs

/* system includes */
#include <iostream>
#include <string>
#include <cstddef>

#include "Parsing.h"

using namespace std;

namespace parsing_tests
{

int test_1(void) {
    std::cout << "Test 1: parsing a correct file." << std::endl;

	string home_str;
	const char* temp = getenv("$HOME");
	if (temp == NULL) {
		cerr << "Could not get $HOME env variable value." << endl;
	} else {
		home_str = string(temp);	
	}
	string fname = "$HOME/control_ws/flights/good_flight.csv"



	bool ret_val = Parsing::flightFromCSV(fname);
	if (!ret_val) {
		std::cerr << "Error parsing the flight file which should be correct." << std::endl;
		return -1;
	} else {
		std::cout << "Successfully parsed file which should be successfully parsed." << std::endl;
		return 0;
	}
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

}

