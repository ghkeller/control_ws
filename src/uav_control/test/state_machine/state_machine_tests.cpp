#include <iostream>
#include <string>
#include <queue>
#include <vector>

#include "StateMachine.h"
#include "ExampleStateMachines.h"

namespace testing
{

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

bool assert_neq(auto a, auto b, string test_desc = "")
{
	if (a != b) {
		cout << "Assert neq: passed >> " << test_desc << endl;
		return true;
	} else {
		cout << "Assert neq: failed >> " << test_desc << endl;
		return false;
	}

}


bool assert_nullptr(auto a, string test_desc = "")
{
	if (a == nullptr) {
		cout << "Assert nullptr: passed >> " << test_desc << endl;
		return true;
	} else {
		cout << "Assert nullptr: failed >> " << test_desc << endl;
		return false;
	}

}

} // namespace testing END

using namespace testing;

namespace method_tests
{
	int getCurrentState_test_1()
	{
		// see if we can get the state from the state machine after instantiating
		TheirStateMachine sm; 

		// check the starting state
		int state = sm.getCurrentState();
		return assert_eq(state,TheirStateMachine::State::STATE_1, "the current state being equal to the starting state"); 
	}


	int getCurrentState_test_2()
	{
		// ensure that we are distinguishing between states
		TheirStateMachine sm; 

		// check the starting state
		int state = sm.getCurrentState();
		return assert_neq(state,TheirStateMachine::State::STATE_2, "the current state not being equal to a different state"); 
	}


	int setCurrentState_test_1()
	{
		// see if we can get the state from the state machine after instantiating
		TheirStateMachine sm; 

		// change the state
		sm.setCurrentState( TheirStateMachine::State::STATE_2 );

		// check the starting state
		int state = sm.getCurrentState();
		return assert_eq( state, TheirStateMachine::State::STATE_2, "the current state being equal to the state we manually set it to" ); 
	}

	int setCurrentState_test_2()
	{
		// see if we can get the state from the state machine after instantiating
		TheirStateMachine sm; 

		// change the state
		sm.setCurrentState( TheirStateMachine::State::STATE_2 );

		// check the starting state
		int state = sm.getCurrentState();
		return assert_eq( state, TheirStateMachine::State::STATE_2, "the current state being not equal to the old state"  ); 
	}
		
		
} // namespace method_tests END


using namespace method_tests;

int main( void )
{

	int result;

    std::cout << "Test 1: testing the getCurrentState method of the sm" << std::endl;
    result = getCurrentState_test_1();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;
	
    std::cout << "Test 2: testing the getCurrentState ret val isn't equal to a state we don't expect to be in" << std::endl;
    result = getCurrentState_test_2();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;

    std::cout << "Test 3: testing the setCurrentState method of the sm" << std::endl;
    result = setCurrentState_test_1();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;
	
    std::cout << "Test 4: making sure that we aren't equal to the old state" << std::endl;
    result = setCurrentState_test_2();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;
	

/*
	// pass in an event to register
	queue<StateMachine::Event *> events_to_register;
	events_to_register.push( new MyStateMachine::Event( MyStateMachine::Event::AN_EVENT ) );
	events_to_register.push( new MyStateMachine::Event( MyStateMachine::Event::NO_EVENT ) );
	sm.registerEvents(events_to_register);

	// do a "check events" (actually will just call in cycle -- cant get to the events which will be in the sub sm's queue)
	TheirStateMachine::Event* event = (TheirStateMachine::Event *) sm.checkEvents();
	if (event != nullptr)
		cout << event->value() << endl; 
	event = (TheirStateMachine::Event *) sm.checkEvents();
	if (event != nullptr)
		cout << event->value() << endl; 
*/
}
