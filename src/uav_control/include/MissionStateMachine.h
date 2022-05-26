#pragma once

/* system includes */
#include <iostream>
#include <string>
#include <cstddef>

/* local includes */

using namespace std;

class MissionStateMachine
{
	public:
	// constructor
	MissionStateMachine();

	// defining states
	enum class State {INIT, ARMING, TAKING_OFF, IN_OFFBOARD, AVOIDING, HOLDING};

	// main execution
	void cycle();

	//getters
	State getCurrentState(void);

	//setters

	private:
	State current_state;
	struct flags {
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
