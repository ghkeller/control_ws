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

/* TESTS AND DESCRIPTIOS

methods:
	- getCurrentState DONE
	- setCurrentState DONE
	- registerEvents DONE
	- isAValidEvent 
	- checkValidEvent
	- checkEvents DONE

main function:
	- cycle
		- state transitions
			- into/out of sub-state machines
			- testing the pointer value of the sub state machine
		- registering events

*/

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

	int registerEvents_test_1()
	{
		// 
		TheirStateMachine sm; 

		// create some event and add to the queue
		queue<StateMachine::Event *> events_to_reg_q; // this exists on the stack, declared up top
		events_to_reg_q .push( new TheirStateMachine::Event( TheirStateMachine::Event::AN_EVENT ) );
		sm.registerEvents( events_to_reg_q );

		// just seeing if we can, not that it was ingested properly (check output)
		return true;
	}

	int registerEvents_test_2()
	{
		// pass an event that will be ingested by the sub-state machine 
		TheirStateMachine sm; 

		// create some event and add to the queue
		queue<StateMachine::Event *> events_to_reg_q; // this exists on the stack, declared up top
		events_to_reg_q .push( new MyStateMachine::Event( MyStateMachine::Event::AN_EVENT ) );
		sm.registerEvents( events_to_reg_q );

		// just seeing if we can, not that it was ingested properly (check output)
		return true;
	}


	int checkEvents_test_1()
	{
		TheirStateMachine sm; 

		// ensure that the event queue starts off empty
		int num_of_events = sm.eventQueueSize();
		return assert_eq( 0, num_of_events, "checking that there aren't any events in the queue to start off with" );
	}

	int checkEvents_test_2()
	{
		TheirStateMachine sm; 

		// same test, but with the eventQueueEmpty method
		bool empty = sm.eventQueueEmpty();
		return assert_eq( true, empty, "checking that there aren't any events in the queue to start off with, again, with different method" );
	}

	int checkEvents_test_3()
	{
		TheirStateMachine sm; 

		// create some event and add to the queue
		queue<StateMachine::Event *> events_to_reg_q; // this exists on the stack, declared up top
		events_to_reg_q .push( new TheirStateMachine::Event( TheirStateMachine::Event::AN_EVENT ) );
		sm.registerEvents( events_to_reg_q );

		// ensure that the event queue now has an event that we need to process
		int num_of_events = sm.eventQueueSize();
		return assert_eq( 1, num_of_events, "checking that there is now only one event in the event_queue" );
	}

	int checkEvents_test_4()
	{
		TheirStateMachine sm; 

		// create some event and add to the queue
		queue<StateMachine::Event *> events_to_reg_q; // this exists on the stack, declared up top
		events_to_reg_q .push( new TheirStateMachine::Event( TheirStateMachine::Event::AN_EVENT ) );
		sm.registerEvents( events_to_reg_q );

		// same test, but with the eventQueueEmpty method
		bool empty = sm.eventQueueEmpty();
		return assert_eq( false, empty, "checking that the queue is no longer empty" );
	}
		
		
} // namespace method_tests END

namespace execution_tests
{

	int STATE_1_to_STATE_2_transition_test_1()
	{
		MyStateMachine sm;

		// cycle the state machine
		sm.cycle();

		// pass an event to the state machine
		queue<StateMachine::Event *> events_to_reg_q; // this exists on the stack, declared up top
		events_to_reg_q .push( new MyStateMachine::Event( MyStateMachine::Event::AN_EVENT ) );
		sm.registerEvents( events_to_reg_q );

		// cycle a couple more times
		sm.cycle();
		sm.cycle();

		// check that we're in the next state now
		int current_state = sm.getCurrentState();
		return assert_eq( current_state, MyStateMachine::State::STATE_2, "Checking that we're now in STATE_2" );
	}

	int STATE_1_to_STATE_2_nontransition_test_1()
	{
		MyStateMachine sm;

		// cycle the state machine
		sm.cycle();

		// pass an event to the state machine
		queue<StateMachine::Event *> events_to_reg_q; // this exists on the stack, declared up top
		events_to_reg_q .push( new TheirStateMachine::Event( TheirStateMachine::Event::AN_EVENT ) );
		sm.registerEvents( events_to_reg_q );

		// cycle a couple more times
		sm.cycle();
		sm.cycle();

		// check that we haven't transitioned state since it was the wrong type of event
		int current_state = sm.getCurrentState();
		return assert_eq( current_state, MyStateMachine::State::STATE_1, "Checking that we've stayed in STATE_1" );
	}


	int STATE_1_to_STATE_2_transition_test_2()
	{
		TheirStateMachine sm;

		// cycle the state machine
		sm.cycle();

		// pass an event to the state machine
		queue<StateMachine::Event *> events_to_reg_q; // this exists on the stack, declared up top
		events_to_reg_q .push( new TheirStateMachine::Event( TheirStateMachine::Event::AN_EVENT ) );
		sm.registerEvents( events_to_reg_q );

		// cycle a couple more times
		sm.cycle();
		sm.cycle();

		// check that we're in the next state now
		int current_state = sm.getCurrentState();
		return assert_eq( current_state, TheirStateMachine::State::STATE_2, "Checking that we're now in STATE_2 for top level sm that has a sub sm" );
	}

	int STATE_1_to_STATE_2_nontransition_test_2()
	{
		TheirStateMachine sm;

		// cycle the state machine
		sm.cycle();

		// pass an event to the state machine
		queue<StateMachine::Event *> events_to_reg_q; // this exists on the stack, declared up top
		events_to_reg_q .push( new MyStateMachine::Event( MyStateMachine::Event::AN_EVENT ) );
		sm.registerEvents( events_to_reg_q );

		// cycle a couple more times
		sm.cycle();
		sm.cycle();

		// check that we haven't transitioned state since it was the wrong type of event
		int current_state = sm.getCurrentState();
		return assert_eq( current_state, TheirStateMachine::State::STATE_1, "Checking that we've stayed in STATE_1" );
	}


	int STATE_1_to_STATE_2_in_sub_sm_transition_test_2()
	{
		TheirStateMachine sm;

		// cycle the state machine
		sm.cycle();

		// pass an event to the state machine
		queue<StateMachine::Event *> events_to_reg_q; // this exists on the stack, declared up top
		events_to_reg_q .push( new TheirStateMachine::Event( TheirStateMachine::Event::AN_EVENT ) );
		sm.registerEvents( events_to_reg_q );

		// cycle a couple more times
		sm.cycle();
		sm.cycle();

		// we're now in STATE_2 which has a sub-state machine that it has and will cycle
		// while in this state. Pass the eveent to make the sub-state machine cycle

		// pass an event to the state machine
		events_to_reg_q .push( new MyStateMachine::Event( MyStateMachine::Event::AN_EVENT ) );
		sm.registerEvents( events_to_reg_q );

		// cycle a couple more times
		sm.cycle();
		sm.cycle();


		// check that we haven't transitioned state since it was the wrong type of event
		int current_state = sm.getCurrentState();
		return assert_eq( current_state, TheirStateMachine::State::STATE_2, "Checking that we're in STATE_2, and check output for sub-sm stuff" );
	}


}


using namespace method_tests;
using namespace execution_tests;

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
	
    std::cout << "Test 5: try to register an event for the top-level state machine" << std::endl;
    result = registerEvents_test_1();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;
	
    std::cout << "Test 6: try to register an event for the sub state machine" << std::endl;
    result = registerEvents_test_2();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;
	
    std::cout << "Test 7: check the event queue size when it should be empty" << std::endl;
    result = checkEvents_test_1();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;
	
    std::cout << "Test 8: check the event queue is empty" << std::endl;
    result = checkEvents_test_2();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;
	
    std::cout << "Test 9: check the event queue has had an event added to it" << std::endl;
    result = checkEvents_test_3();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;
	
    std::cout << "Test 10: check the event queue is not empty" << std::endl;
    result = checkEvents_test_4();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;
	
    std::cout << "Test 11: test transitioning states in a top-level sm" << std::endl;
    result = STATE_1_to_STATE_2_transition_test_1();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;
	
    std::cout << "Test 11: test not transitioning states when an event for a different sm with same name passed" << std::endl;
    result = STATE_1_to_STATE_2_nontransition_test_1();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;

    std::cout << "Test 12: transition states in a sub-sm" << std::endl;
    result = STATE_1_to_STATE_2_transition_test_2();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;

    std::cout << "Test 13: test that we will not transition in an sm with a sub sm with event for sub sm passed" << std::endl;
    result = STATE_1_to_STATE_2_nontransition_test_2();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;

    std::cout << "Test 14: try to cycle through a state of the sub-sm" << std::endl;
	result = STATE_1_to_STATE_2_in_sub_sm_transition_test_2();
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
