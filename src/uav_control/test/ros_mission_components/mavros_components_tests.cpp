// mission_state_machine_tests.cpp: test suite for the primary state machine

/* system includes */
#include <iostream>
#include <string>
#include <cstddef>
#include <time.h>
#include <queue>

#include "MAVROSComponents"

using namespace std;

bool assert_true(bool statement, string test_desc = "")
{
	if (statement == true) {
		cout << "Assert true: passed >> " << test_desc << endl;
		return true;
	} else {
		cout << "Assert true: failed >> " << test_desc << endl;
		return true;
	}
}

bool assert_false(bool statement, string test_desc = "")
{
	if (statement == false) {
		cout << "Assert false: passed >> " << test_desc  << endl;
		return true;
	} else {
		cout << "Assert false: failed >> " << test_desc  << endl;
		return false;
	}

}

bool assert_eq(auto a, auto b, string test_desc = "")
{
	if (a == b) {
		cout << "Assert eq: passed >> " << test_desc << endl;
		return true;
	} else {
		cout << "Assert eq: failed >> " << test_desc << endl;
		return false;
	}

}

namespace mavros_component_tests
{

bool test_1(void) {
    cout << "Test 1: instantiating a mavros components object" << endl;

	// instantiate the state machine
	MAVROSComponents comp = MAVROSComponents ();
}

} // namespace mission_state_machine_tests END

using namespace mavros_component_tests;

int main(void)
{
    bool result; // stores the results of the test

    std::cout << "Starting the mavros components tests..." << std::endl;

    std::cout << "Test 1:" << std::endl;
    result = test_1();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;
}

