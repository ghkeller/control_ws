#pragma once

#include <iostream>
#include <string>
#include <queue>
#include <vector>

#include "StateMachine.h"
/* EXAMPLE DERIVED */

class MyStateMachine : public StateMachine
{
	public:

	class State : public StateMachine::State {
		public:
		// pseudo-enum declaration
		static const int STATE_1 = 0;
		static const int STATE_2 = 1;

		State ();
		State ( int starting_state ) { this->value() = starting_state; };
	};

	class Event : public StateMachine::Event {
		public:
		// pseudo-enum declaration
		static const int NO_EVENT = 0;
		static const int AN_EVENT = 1;

		Event ();
		Event ( int event_type ) { this->value() = event_type; };
	};

	MyStateMachine () : StateMachine {} { this->current_state_ptr = new State( State::STATE_1 ); }
	virtual ~MyStateMachine() {};

	bool isAValidEvent( StateMachine::Event* event_ptr ) { return checkValidEvent<MyStateMachine::Event *>( event_ptr ); };
	void cycle();

};

class TheirStateMachine : public StateMachine
{
	public:

	class State : public StateMachine::State {
		public:
		// pseudo-enum declaration
		static const int STATE_1 = 0;
		static const int STATE_2 = 1;

		State ();
		State ( int starting_state ) { this->value() = starting_state; };
	};

	class Event : public StateMachine::Event {
		public:
		// pseudo-enum declaration
		static const int NO_EVENT = 0;
		static const int AN_EVENT = 1;
		static const int WOW_EVENT = 2;

		Event ();
		Event ( int event_type ) { this->value() = event_type; };
	};

	TheirStateMachine () : StateMachine {} {
		this->current_state_ptr = new State( State::STATE_1 );
		this->sub_SMs.push_back( &( this->mySubSM ) );
	}
	virtual ~TheirStateMachine() {};
	void cycle();

	bool isAValidEvent( StateMachine::Event* event_ptr ) { return checkValidEvent<TheirStateMachine::Event *>( event_ptr ); };

	private:
	MyStateMachine mySubSM = MyStateMachine();

};

