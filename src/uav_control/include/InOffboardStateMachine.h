#pragma once

#include <queue>

#include "StateMachine.h"
#include "Timer.h"

enum class InOffboardEvents { WAYPOINT_HIT, POST_WP_HIT_TIMER_STARTED, POST_WP_HIT_TIMER_FINISHED };

class InOffboardEvent : public Event {
	public: 
	InOffboardEvent() {};
	virtual ~InOffboardEvent() {};
	InOffboardEvent( InOffboardEvents event ) { this->val = event; };

	// value getter
	InOffboardEvents getValue() { return this->val; };

	// value setter
	void setValue(InOffboardEvents val) { this->val = val; };

	private:
	InOffboardEvents val;
};

enum class InOffboardStates { INIT, SETTING_TARGET, CYCLING, STALLING_POST_WP_HIT };

class InOffboardState : public State {
	public: 
	InOffboardState() {};
	virtual ~InOffboardState() {};
	InOffboardState( InOffboardStates state ) { this->val = state; };

	// value getter
	InOffboardStates getValue() { return this->val; };

	// value setter
	void setValue(InOffboardStates val) { this->val = val; };

	private:
	InOffboardStates val;
};

class InOffboardStateMachine : public StateMachine
{
	public:
	InOffboardStateMachine() {};

	virtual void cycle();

	// override for determining event type
	virtual bool isAValidEvent(Event* event) { return checkValidEvent<InOffboardEvent *>( event ); };

	private:
	InOffboardState current_state = InOffboardState( InOffboardStates::INIT );
	Timer waypoint_hit_stall_timer;
};
