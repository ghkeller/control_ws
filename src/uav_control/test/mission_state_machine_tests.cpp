// mission_state_machine_tests.cpp: test suite for the primary state machine

/* system includes */
#include <iostream>
#include <string>
#include <cstddef>
#include <time.h>
#include <queue>

#include "StateMachine.h"
#include "MissionStateMachine.h"
//#include "InOffboardStateMachine.h"

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

/*
void cycleStateMachineSeveralTimes(StateMachine* state_machine)
{
	// cycle the state machine several times
	// Use current time as seed for random generator
	srand(time(0));

	int cycleAmt = rand() % 20;
	for(int i = 0; i<cycleAmt; i++)
		state_machine->cycle();
}
*/

namespace mission_state_machine_tests
{

bool test_1(void) {
    cout << "Test 1: instantiating the state machine" << endl;

	// instantiate the state machine
	MissionStateMachine state_machine = MissionStateMachine();
	MissionStates current_state = state_machine.getCurrentState();

	// check if we start in the INIT state
	return assert_eq(current_state, MissionStates::INIT, "Checking that we're in INIT on initialization.");
}


bool test_2(void) {
    cout << "Test 2: transitioning out of the INIT state and into the CHECKING_PREARM state" << endl;

	// instantiate the state machine
	MissionStateMachine state_machine = MissionStateMachine();

	// cycling the state machine once will take us out of init (i.e., nothing needs to happen externally)
	state_machine.cycle();
	MissionStates current_state = state_machine.getCurrentState();

	return assert_eq(current_state, MissionStates::CHECKING_PREARM, "Checking that we have transitioned into the CHECKING_PREARM state");
}


bool test_3(void) {
    cout << "Test 3: testing state after checks completed" << endl;

	// instantiate the state machine
	MissionStateMachine state_machine = MissionStateMachine();

	// cycling the state machine once will take us out of init (i.e., nothing needs to happen externally)
	state_machine.cycle();

	// cycling the state machine again should still keep us in the PREARM_CHECKING state"
	state_machine.cycle();

	MissionEvent e( MissionEvents::CHECKS_PREARM_COMPLETE );
	queue<Event *> q( { &e } );

	state_machine.registerEvents( q );
	state_machine.cycle();

	MissionStates current_state = state_machine.getCurrentState();

	return assert_eq(current_state, MissionStates::ARMING, "Checking that we have not transitioned -- we have only cycled, but no event has been registered yet.");
}

bool test_5(void) {
    cout << "Test 5: testing the setCurrentState function" << endl;

	// instantiate the state machine
	MissionStateMachine state_machine = MissionStateMachine();

	state_machine.setCurrentState( MissionStates::ARMING );

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
	return assert_eq(state, MissionStateMachine::State::EXIT, "Checking that we have transitioned through the whole state machine.");
}

bool test_13(void) {
    cout << "Test 13: initialize the sub-state machine for the offboard state" << endl;

	// instantiate the state machine without pre-setting state
	InOffboardStateMachine offboard_state_machine = InOffboardStateMachine();

	// check to make sure that we start in init
	InOffboardStateMachine::State state = offboard_state_machine.getCurrentState();
	return assert_eq(state, InOffboardStateMachine::State::INIT, "Check to make sure we start in the init state");
}


bool test_14(void) {
    cout << "Test 14: transitioning out of the INIT state and into the SETTING_TARGET state of the sub sm" << endl;

	// instantiate the state machine without pre-setting state
	InOffboardStateMachine offboard_state_machine = InOffboardStateMachine();

	// cycle once to get out of the init state
	offboard_state_machine.cycle();

	// check to make sure that we start in init
	InOffboardStateMachine::State state = offboard_state_machine.getCurrentState();
	return assert_eq(state, InOffboardStateMachine::State::SETTING_TARGET, "Checking that we have transitioned into the SETTING_TARGET state");
}

bool test_15(void) {
    cout << "Test 15: transitioning out of the SETTING_TARGET state and into the CYCLING state of the sub sm" << endl;

	// instantiate the state machine without pre-setting state
	InOffboardStateMachine offboard_state_machine = InOffboardStateMachine();
	offboard_state_machine.setCurrentState(InOffboardStateMachine::State::SETTING_TARGET);

	// cycle once to get out of the init state
	offboard_state_machine.cycle();

	// check to make sure that we start in init
	InOffboardStateMachine::State state = offboard_state_machine.getCurrentState();
	return assert_eq(state, InOffboardStateMachine::State::CYCLING, "Checking that we have transitioned into the CYCLING state");
}

bool test_16(void) {
    cout << "Test 16: transitioning out of the CYCLING state and into the STALLING_POST_WP_HIT state of the sub sm" << endl;

	// instantiate the state machine without pre-setting state
	InOffboardStateMachine offboard_state_machine = InOffboardStateMachine();
	offboard_state_machine.setCurrentState(InOffboardStateMachine::State::CYCLING);

	// cycle once to get out of the init state
	offboard_state_machine.cycle();

	// let the sm know that we've hit the waypoint
	offboard_state_machine.registerEvent(InOffboardStateMachine::Event::WAYPOINT_HIT);

	// cycle once to process the event registered
	offboard_state_machine.cycle();

	// check to make sure that we start in init
	InOffboardStateMachine::State state = offboard_state_machine.getCurrentState();
	return assert_eq(state, InOffboardStateMachine::State::STALLING_POST_WP_HIT , "Checking that we have transitioned into the STALLING_POST_WP_HIT state");
}


bool test_17(void) {
    cout << "Test 17: transitioning out of the STALLING_POST_WP_HIT state and back into the state of SETTING_TARGET (i.e., there are more waypoints) in the sub sm" << endl;

	// instantiate the state machine without pre-setting state
	InOffboardStateMachine offboard_state_machine = InOffboardStateMachine();
	offboard_state_machine.setCurrentState(InOffboardStateMachine::State::STALLING_POST_WP_HIT);

	// cycle once to get out of the init state
	offboard_state_machine.cycle();

	// let the sm know that the timer has elapsed -- this is fudged, because in this case, it has not actually elapsed, but we want to test the transition
	offboard_state_machine.registerEvent(InOffboardStateMachine::Event::WAYPOINT_STALL_TIMER_ELAPSED);

	// cycle once to process the event registered
	offboard_state_machine.cycle();

	// check to make sure that we start in init
	InOffboardStateMachine::State state = offboard_state_machine.getCurrentState();
	return assert_eq(state, InOffboardStateMachine::State::SETTING_TARGET, "Checking that we have transitioned into the STALLING_POST_WP_HIT state");
}

*/
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

	/*

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

    std::cout << "Test 13:" << std::endl;
    result = test_13();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;

    std::cout << "Test 14:" << std::endl;
    result = test_14();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;

    std::cout << "Test 15:" << std::endl;
    result = test_15();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;

    std::cout << "Test 16:" << std::endl;
    result = test_16();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;

    std::cout << "Test 17:" << std::endl;
    result = test_17();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;
*/
}

