#pragma once

/* system includes */
#include <iostream>
#include <string>
#include <cstddef>
#include <queue>

/* local includes */

using namespace std;

class MissionStateMachine
{
	public:
	// constructor
	MissionStateMachine();

	// defining enums
	enum class State {INIT, CHECKING_PREARM, ARMING, TAKING_OFF, IN_OFFBOARD, AVOIDING, HOLDING};
	enum class Event {CHECKS_PREARM_COMPLETE, NO_EVENT};

	// main execution and state machine interfacing
	void cycle();
	Event checkEvents();
	void registerEvent(Event);

	//getters
	State getCurrentState(void);

	//setters
	void setCurrentState(State);

	private:
	State current_state;
	queue<Event> event_queue;
	struct transition_flags {
		bool state_entry;
		bool state_exit;
	} flags;
};

/*
class InOffboardSubSM
{
	public:
	// constructor
	InOffboardSubSM();

	// defining states
	enum Class state {SUB_INIT, SETTING_TARGET, CYCLING} offboard_substate;

	// main execution
	cycle();

	private:
	State next_state;
}
*/
