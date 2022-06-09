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


bool assert_nullptr(auto a, string test_desc = "")
{
	if (a == nullptr) {
		cout << "Assert eq: passed >> " << test_desc << endl;
		return true;
	} else {
		cout << "Assert eq: failed >> " << test_desc << endl;
		return false;
	}

}

} // namespace testing END

namespace method_tests
{
	int getCurrentState_test_1()
	{
		// see if we can get the state from the state machine after instantiating
		TheirStateMachine sm; 

		// check the starting state
		TheirStateMachine::State* state = (TheirStateMachine::State *) sm.getCurrentState();
		cout << state->value() << endl;
	}
		
} // namespace method_tests END

int main( void )
{

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
}
