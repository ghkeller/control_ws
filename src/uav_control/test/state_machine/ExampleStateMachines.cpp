#include <iostream>
#include <string>
#include <queue>
#include <vector>

#include "StateMachine.h"
#include "ExampleStateMachines.h"

#define DEBUG

#define BLACK 30
#define RED 31
#define GREEN 32
#define YELLOW 33
#define BLUE 34
#define MAGENTA 35
#define CYAN 36
#define WHITE 37

using namespace std;

void debugOut(string str, int color_code)
{
#ifdef DEBUG
	cout << "\033[1;" << color_code << "m" << str << "\033[0m\n";
#endif
}

void MyStateMachine::cycle()
{
	// set up next state as current in case nothing changes
	int current_state = this->getCurrentState();
	int next_state = current_state;


	// check for events to ingest
	Event* event_ptr = (Event*) this->checkEvents();
	int event;
	if (event_ptr == nullptr)
		event = Event::NO_EVENT;
	else
		event = event_ptr->value();

	switch (current_state)
	{

	case State::STATE_1:

		if (this->flags.state_entry == true) {
			// state entry execution
			debugOut("	In state 'STATE_1'...", BLUE);
			this->flags.state_entry = false;
		}

		// STATE TRANSFER CONDITIONS 

		if (event == MyStateMachine::Event::AN_EVENT) {
			debugOut(" The event happened. ", YELLOW);
			debugOut("	Next state will be 'STATE_2'...", CYAN);
			next_state = MyStateMachine::State::STATE_2;
			this->flags.state_exit = true;
		}

		if (this->flags.state_exit == true) {
			debugOut("	Exiting 'STATE_1'...", MAGENTA);
			cout << endl;
			this->flags.state_exit = false;
			this->flags.state_entry = true;
		}
	
	break;

	case State::STATE_2:

		if (this->flags.state_entry == true) {
			// state entry execution
			debugOut("	In state 'STATE_2'...", BLUE);
			this->flags.state_entry = false;
		}

		// STATE TRANSFER CONDITIONS 

		if (this->flags.state_exit == true) {
			debugOut("	Exiting 'STATE_2'...", MAGENTA);
			cout << endl;
			this->flags.state_exit = false;
			this->flags.state_entry = true;
		}
	
	break;


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


