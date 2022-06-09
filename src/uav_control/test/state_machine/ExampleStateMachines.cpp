#include <iostream>
#include <string>
#include <queue>
#include <vector>

#include "StateMachine.h"
#include "ExampleStateMachines.h"

void MyStateMachine::cycle()
{
	// set up next state as current in case nothing changes
	int current_state = this->getCurrentState();
	int next_state = current_state;


	// check for events to ingest
	Event* event = (Event*) this->checkEvents();

	switch (current_state)
	{
	default:

	break;
	}

	// make state change here
	this->setCurrentState( next_state );
}


void TheirStateMachine::cycle()
{
	// set up next state as current in case nothing changes
	int current_state = this->getCurrentState();
	int next_state = current_state;


	// check for events to ingest
	Event* event = (Event*) this->checkEvents();

	switch (current_state)
	{
	default:

	break;
	}

	// make state change here
	this->setCurrentState( next_state );
}


