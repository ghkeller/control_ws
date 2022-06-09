/* system includes */
#include <iostream>
#include <string>
#include <queue>

/* local includes */
#include "StateMachine.h"

// constructor
StateMachine::StateMachine() {
	// initialize the necessary components for our state machine

	this->flags.state_entry = true;
	this->flags.state_exit = false;
}

State* StateMachine::getCurrentState() {
	return this->current_state_ptr;
}


Event* StateMachine::checkEvents()
{
	// if no events, return NO_EVENT
	if (this->event_ptr_queue.empty()) {
		return nullptr; // might have to give this a special value so that we can  use it indescriminately
	} 

	cout << "here 1" << endl;

	// pop and return the event
	Event* event_ptr = this->event_ptr_queue.front();
	this->event_ptr_queue.pop();
	cout << "here 2" << endl;
	return event_ptr;
}

void StateMachine::registerEvents( queue<Event*>& events_to_register_q )
{
	// add events to this state machine to be processed
	queue<Event*> unprocessed_events;
	while ( !events_to_register_q.empty() )
	{
		cout << "processing an event..." << endl;
		Event* event_ptr = events_to_register_q.front();
		if ( this->isAValidEvent( event_ptr ) ) {
			cout << "	... adding this event to the locally maintained queue." << endl;
			this->event_ptr_queue.push( event_ptr );
		} else {
			cout << "	...event is not valid for this sm. passing to substates..." << endl;
			unprocessed_events.push( event_ptr );
		}
		events_to_register_q.pop();
	}


	// pass unprocessed events to the substate machines
	for ( StateMachine * sub_state_machine : this->sub_state_machines )
	{ 
		sub_state_machine->registerEvents( unprocessed_events );
	}
}
