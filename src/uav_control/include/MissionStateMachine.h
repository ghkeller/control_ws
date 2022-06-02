#pragma once

/* system includes */
#include <iostream>
#include <string>
#include <cstddef>
#include <queue>
#include <map>

/* local includes */
#include "Timer.h"

using namespace std;


class MissionStateMachine
{
	public:
	// constructor
	MissionStateMachine();

	// defining enums
	enum class State {INIT, CHECKING_PREARM, ARMING, TAKING_OFF, IN_OFFBOARD, AVOIDING, HOLDING, RETURNING_TO_HOME, LANDING, DISARMING, EXIT};
	enum class Event {CHECKS_PREARM_COMPLETE, AIRCRAFT_ARMED, TAKEOFF_COMPLETE, OFFBOARD_MISSION_COMPLETE, REACHED_HOME_COORDS, TOUCHED_DOWN, DISARMED, NO_EVENT};

	// mappings for names
	map<State, string> state_map;

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


class InOffboardStateMachine
{
	public:
	// constructor
	InOffboardStateMachine();

	// defining states
	enum class State {INIT, SETTING_TARGET, CYCLING, STALLING_POST_WP_HIT}; 
	enum class Event {WAYPOINT_HIT, POST_WP_HIT_TIMER_STARTED, POST_WP_HIT_TIMER_FINISHED, NO_EVENT};

	// mappings for names
	map<State, string> state_map;

	// main execution and state machine interfacing
	void cycle();
	Event checkEvents();
	void registerEvent(Event);

	//getters
	State getCurrentState(void);

	//setters
	void setCurrentState(State);

	private:
	Timer waypoint_hit_stall_timer;
	State current_state;
	queue<Event> event_queue;
	struct transition_flags {
		bool state_entry;
		bool state_exit;
	} flags;
};
