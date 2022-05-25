// mission_state_machine_tests.cpp: test suite for the primary state machine

/* system includes */
#include <iostream>
#include <string>
#include <cstddef>

#include "MissionStateMachine.h"

using namespace std;

bool assert_true(bool statement, string test_desc = "")
{
	if (statement == true) {
		cout << "Assert true: passed >>" << endl;
		return true;
	} else {
		cout << "Assert true: failed >>" << endl;
		return true;
	}

}

bool assert_false(bool statement, string test_desc = "")
{
	if (statement == false) {
		cout << "Assert false: passed >>" << endl;
		return true;
	} else {
		cout << "Assert false: failed >>" << endl;
		return false;
	}

}

bool assert_eq(auto a, auto b, string test_desc = "")
{
	if (a == b) {
		cout << "Assert eq: passed >>" << endl;
		return true;
	} else {
		cout << "Assert eq: failed >>" << endl;
		return false;
	}

}

namespace mission_state_machine_tests
{

int test_1(void) {
    cout << "Test 1: instantiating the state machine" << endl;

	// instantiate the state machine
	MissionStateMachine state_machine = MissionStateMachine();

	MissionStateMachine::State state = state_machine->getCurrentState()
	cout << "Current state: " << string(state) << endl;
	assert_eq(state, MissionStateMachine::State::INIT);
}

int main(void)
{
    int result; // stores the results of the test

    std::cout << "Starting the mission state machine tests..." << std::endl;

    std::cout << "Test 1:" << std::endl;
    result = test_1();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }

}

