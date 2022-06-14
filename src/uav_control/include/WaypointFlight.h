#pragma once

#include <iostream>
#include <string>
#include <queue>
#include <vector>
#include <ros/ros.h>

#include "Flight.h"
#include "StateMachine.h"


/*
class SubSM : public StateMachine
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

	SubSM () : StateMachine {} { this->current_state_ptr = new State( State::STATE_1 ); }
	virtual ~SubSM() {};

	bool isAValidEvent( StateMachine::Event* event_ptr ) { return checkValidEvent<SubSM::Event *>( event_ptr ); };
	void cycle();
};
*/


class WaypointFlight : public Flight
{
	public:

	class State : public StateMachine::State {
		public:
		// pseudo-enum declaration
		static const int INIT = 0;
		static const int CHECKING_PREARM = 1;
		static const int ARMING = 2;

		State ();
		State ( int starting_state ) { this->value() = starting_state; };
	};

	class Event : public StateMachine::Event {
		public:
		// pseudo-enum declaration
		static const int NO_EVENT = 0;
		static const int CHECKS_PREARM_COMPLETE = 1;

		Event ();
		Event ( int event_type ) { this->value() = event_type; };
	};

	WaypointFlight () : Flight {} { this->current_state_ptr = new State( State::INIT ); }
	virtual ~WaypointFlight() {};

	bool isAValidEvent( StateMachine::Event* event_ptr ) { return checkValidEvent<WaypointFlight::Event *>( event_ptr ); };
	void cycle();

	private:

};

