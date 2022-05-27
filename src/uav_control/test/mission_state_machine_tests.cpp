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
		cout << "Assert eq: passed >>" << test_desc << endl;
		return true;
	} else {
		cout << "Assert eq: failed >>" << test_desc << endl;
		return false;
	}

}

namespace mission_state_machine_tests
{

bool test_1(void) {
    cout << "Test 1: instantiating the state machine" << endl;

	// instantiate the state machine
	MissionStateMachine state_machine = MissionStateMachine();

	MissionStateMachine::State state = state_machine.getCurrentState();
	return assert_eq(state, MissionStateMachine::State::INIT, "Checking that we're in INIT on initialization.");
}


bool test_2(void) {
    cout << "Test 2: transitioning out of the INIT state and into the CHECKING state" << endl;

	// instantiate the state machine
	MissionStateMachine state_machine = MissionStateMachine();

	// cycling the state machine once will take us out of init (i.e., nothing needs to happen externally)
	state_machine.cycle();
	MissionStateMachine::State state = state_machine.getCurrentState();

	return assert_eq(state, MissionStateMachine::State::CHECKING_PREARM, "Checking that we have transitioned into the CHECKING_PREARM state");
}


bool test_3(void) {
    cout << "Test 3: cycling, but staying in the CYCLING state" << endl;

	// instantiate the state machine
	MissionStateMachine state_machine = MissionStateMachine();

	// cycling the state machine once will take us out of init (i.e., nothing needs to happen externally)
	state_machine.cycle();

	// cycling the state machine again should still keep us in the PREARM_CHECKING state"
	state_machine.cycle();

	MissionStateMachine::State state = state_machine.getCurrentState();
	return assert_eq(state, MissionStateMachine::State::CHECKING_PREARM, "Checking that we have not transitioned -- we have only cycled, but no event has been registered yet.");
}

bool test_4(void) {
    cout << "Test 4: transitioning out of the INIT state and into the CHECKING state" << endl;

	// instantiate the state machine
	MissionStateMachine state_machine = MissionStateMachine();

	// cycling the state machine once will take us out of init (i.e., nothing needs to happen externally)
	state_machine.cycle();

	// let the state machine know that everything we need to do for checking has been completed
	state_machine.registerEvent(MissionStateMachine::Event::CHECKS_PREARM_COMPLETE);

	// cycling the state machine once will take us out of init (i.e., nothing needs to happen externally)
	state_machine.cycle();

	// check to make sure that we will now transition into ARMING
	MissionStateMachine::State state = state_machine.getCurrentState();
	return assert_eq(state, MissionStateMachine::State::ARMING, "Checking that we have transitioned into the ARMING state");
}

} // namespace mission_state_machine_tests END

using namespace mission_state_machine_tests;

int main(void)
{
    bool result; // stores the results of the test

    std::cout << "Starting the mission state machine tests..." << std::endl;

    std::cout << "Test 1:" << std::endl;
    result = test_1();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }

    std::cout << "Test 2:" << std::endl;
    result = test_2();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }

    std::cout << "Test 3:" << std::endl;
    result = test_3();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }

    std::cout << "Test 4:" << std::endl;
    result = test_4();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }

}

