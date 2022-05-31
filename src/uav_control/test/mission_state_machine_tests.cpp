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

bool test_5(void) {
    cout << "Test 5: testing the setCurrentState function" << endl;

	// instantiate the state machine
	MissionStateMachine state_machine = MissionStateMachine();
	state_machine.setCurrentState(MissionStateMachine::State::ARMING);

	// cycling the state machine once to make sure we've actually dropped into that state
	// this should show debug output as evidence of this 
	state_machine.cycle();

	// check to make sure that we will now transition into ARMING
	MissionStateMachine::State state = state_machine.getCurrentState();
	return assert_eq(state, MissionStateMachine::State::ARMING, "Checking that we have transitioned into the ARMING state");
}

bool test_6(void) {
    cout << "Test 6: transitioning out of ARMING, into TAKEOFF" << endl;

	// instantiate the state machine
	MissionStateMachine state_machine = MissionStateMachine();
	state_machine.setCurrentState(MissionStateMachine::State::ARMING);

	// cycling the state machine once to make sure we've actually dropped into that state
	// this should show debug output as evidence of this 
	state_machine.cycle();

	// register that we are now armed
	state_machine.registerEvent(MissionStateMachine::Event::AIRCRAFT_ARMED);

	// cycle the machine to consume the event, and we should transition into the takeoff state
	state_machine.cycle();
	state_machine.cycle();

	// check to make sure that we will now transition into ARMING
	MissionStateMachine::State state = state_machine.getCurrentState();
	return assert_eq(state, MissionStateMachine::State::TAKING_OFF, "Checking that we have transitioned into the TAKING_OFF state");
}

bool test_7(void) {
    cout << "Test 7: transitioning out of TAKEOFF, into IN_OFFBOARD" << endl;

	// instantiate the state machine
	MissionStateMachine state_machine = MissionStateMachine();
	state_machine.setCurrentState(MissionStateMachine::State::TAKING_OFF);

	// cycling the state machine once to make sure we've actually dropped into that state
	// this should show debug output as evidence of this 
	state_machine.cycle();

	// register that we are now armed
	state_machine.registerEvent(MissionStateMachine::Event::TAKEOFF_COMPLETE);

	// cycle the machine to consume the event, and we should transition into the takeoff state
	state_machine.cycle();
	state_machine.cycle();

	// check to make sure that we will now transition into ARMING
	MissionStateMachine::State state = state_machine.getCurrentState();
	return assert_eq(state, MissionStateMachine::State::IN_OFFBOARD, "Checking that we have transitioned into the IN_OFFBOARD state");
}

bool test_8(void) {
    cout << "Test 8: transitioning out of IN_OFFBOARD, into RETURNING_TO_HOME" << endl;

	// instantiate the state machine
	MissionStateMachine state_machine = MissionStateMachine();
	state_machine.setCurrentState(MissionStateMachine::State::IN_OFFBOARD);

	// cycling the state machine once to make sure we've actually dropped into that state
	// this should show debug output as evidence of this 
	state_machine.cycle();

	// register that we are now armed
	state_machine.registerEvent(MissionStateMachine::Event::OFFBOARD_MISSION_COMPLETE);

	// cycle the machine to consume the event, and we should transition into the takeoff state
	state_machine.cycle();
	state_machine.cycle();

	// check to make sure that we will now transition into ARMING
	MissionStateMachine::State state = state_machine.getCurrentState();
	return assert_eq(state, MissionStateMachine::State::RETURNING_TO_HOME, "Checking that we have transitioned into the IN_OFFBOARD state");
}

bool test_9(void) {
    cout << "Test 9: transitioning out of RETURNING_TO_HOME, into LANDING" << endl;

	// instantiate the state machine
	MissionStateMachine state_machine = MissionStateMachine();
	state_machine.setCurrentState(MissionStateMachine::State::RETURNING_TO_HOME);

	// cycling the state machine once to make sure we've actually dropped into that state
	// this should show debug output as evidence of this 
	state_machine.cycle();

	// register that we are now armed
	state_machine.registerEvent(MissionStateMachine::Event::REACHED_HOME_COORDS);

	// cycle the machine to consume the event, and we should transition into the takeoff state
	state_machine.cycle();
	state_machine.cycle();

	// check to make sure that we will now transition into ARMING
	MissionStateMachine::State state = state_machine.getCurrentState();
	return assert_eq(state, MissionStateMachine::State::LANDING, "Checking that we have transitioned into the IN_OFFBOARD state");
}

bool test_10(void) {
    cout << "Test 10: transitioning out of LANDING, into DISARMING" << endl;

	// instantiate the state machine
	MissionStateMachine state_machine = MissionStateMachine();
	state_machine.setCurrentState(MissionStateMachine::State::LANDING);

	// cycling the state machine once to make sure we've actually dropped into that state
	// this should show debug output as evidence of this 
	state_machine.cycle();

	// register that we are now armed
	state_machine.registerEvent(MissionStateMachine::Event::TOUCHED_DOWN);

	// cycle the machine to consume the event, and we should transition into the takeoff state
	state_machine.cycle();
	state_machine.cycle();

	// check to make sure that we will now transition into ARMING
	MissionStateMachine::State state = state_machine.getCurrentState();
	return assert_eq(state, MissionStateMachine::State::DISARMING, "Checking that we have transitioned into the IN_OFFBOARD state");
}

bool test_11(void) {
    cout << "Test 11: transitioning out of DISARMING, into EXIT" << endl;

	// instantiate the state machine
	MissionStateMachine state_machine = MissionStateMachine();
	state_machine.setCurrentState(MissionStateMachine::State::DISARMING);

	// cycling the state machine once to make sure we've actually dropped into that state
	// this should show debug output as evidence of this 
	state_machine.cycle();

	// register that we are now armed
	state_machine.registerEvent(MissionStateMachine::Event::DISARMED);

	// cycle the machine to consume the event, and we should transition into the takeoff state
	state_machine.cycle();
	state_machine.cycle();

	// check to make sure that we will now transition into ARMING
	MissionStateMachine::State state = state_machine.getCurrentState();
	return assert_eq(state, MissionStateMachine::State::EXIT, "Checking that we have transitioned into the IN_OFFBOARD state");
}


// all of the events that should happen: enum class Event {CHECKS_PREARM_COMPLETE, AIRCRAFT_ARMED, TAKEOFF_COMPLETE, OFFBOARD_MISSION_COMPLETE, REACHED_HOME_COORDS, TOUCHED_DOWN, DISARMED, NO_EVENT};
bool test_12(void) {
    cout << "Test 12: cycle throught the whole state machine with capped, pseudo-random numbers of cycles between events happening" << endl;

	// instantiate the state machine without pre-setting state
	MissionStateMachine state_machine = MissionStateMachine();
	// we'll start in init
	cycleStateMachineSeveralTimes(state_machine);
	state_machine.registerEvent(MissionStateMachine::Event::CHECKS_PREARM_COMPLETE);
	// now should be in arming
	cycleStateMachineSeveralTimes(state_machine);
	state_machine.registerEvent(MissionStateMachine::Event::AIRCRAFT_ARMED);
	// now should be in taking off
	cycleStateMachineSeveralTimes(state_machine);
	state_machine.registerEvent(MissionStateMachine::Event::TAKEOFF_COMPLETE);
	// now we're in offboard, flyin our mission
	cycleStateMachineSeveralTimes(state_machine);
	state_machine.registerEvent(MissionStateMachine::Event::OFFBOARD_MISSION_COMPLETE);
	// we've completed the mission, so now we're returning home
	cycleStateMachineSeveralTimes(state_machine);
	state_machine.registerEvent(MissionStateMachine::Event::REACHED_HOME_COORDS);
	// we're hovering above the home point now -- time to land
	cycleStateMachineSeveralTimes(state_machine);
	state_machine.registerEvent(MissionStateMachine::Event::TOUCHED_DOWN);
	// we've landed, so let's disarm
	cycleStateMachineSeveralTimes(state_machine);
	state_machine.registerEvent(MissionStateMachine::Event::DISARMED);
	// we're now disarmed, so we're done.
	cycleStateMachineSeveralTimes(state_machine);
	// check to make sure that we've transitioned into EXIT 
	MissionStateMachine::State state = state_machine.getCurrentState();
	return assert_eq(state, MissionStateMachine::State::EXIT, "Checking that we have transitioned into the IN_OFFBOARD state");
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
	std::cout << std::endl;

    std::cout << "Test 2:" << std::endl;
    result = test_2();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;

    std::cout << "Test 3:" << std::endl;
    result = test_3();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;

    std::cout << "Test 4:" << std::endl;
    result = test_4();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;

    std::cout << "Test 5:" << std::endl;
    result = test_5();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;

    std::cout << "Test 6:" << std::endl;
    result = test_6();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;

    std::cout << "Test 7:" << std::endl;
    result = test_7();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;

    std::cout << "Test 8:" << std::endl;
    result = test_8();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;

    std::cout << "Test 9:" << std::endl;
    result = test_9();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;

    std::cout << "Test 10:" << std::endl;
    result = test_10();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;

    std::cout << "Test 11:" << std::endl;
    result = test_11();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;

    std::cout << "Test 12:" << std::endl;
    result = test_12();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;


}

