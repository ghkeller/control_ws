#pragma once

/* system includes */
#include <iostream>
#include <string>
#include <cstddef>

/* local includes */

class FlyMissionSM
{
	public:
	// constructor
	FlyMissionSM();

	// defining states
	enum Class State {INIT, ARMING, TAKING_OFF, IN_OFFBOARD, AVOIDING, HOLDING};

	// main execution
	cycle();

	private:
	State current_state;
	struct flags {
		bool state_entry,
		bool state_exit
	} flags;
}


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
