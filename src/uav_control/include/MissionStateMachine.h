#pragma once

/* system includes */
#include <iostream>
#include <string>
#include <cstddef>
#include <queue>
#include <map>

/* local includes */
#include "StateMachine.h"
//#include "InOffboardStateMachine.h"

using namespace std;

enum class MissionEvents { CHECKS_PREARM_COMPLETE, AIRCRAFT_ARMED, TAKEOFF_COMPLETE, OFFBOARD_MISSION_COMPLETE, REACHED_HOME_COORDS, TOUCHED_DOWN, DISARMED, NO_EVENT };

class MissionEvent : public Event {
	public: 
	MissionEvent () {};
	virtual ~MissionEvent () {};
	MissionEvent ( MissionEvents event ) { this->val = event; };

	// value getter
	MissionEvents getValue() { return this->val; };

	// value setter
	virtual void setValue(MissionEvents val) { this->val = val; };

	private:
	MissionEvents val;
};


enum class MissionStates {INIT, CHECKING_PREARM, ARMING, TAKING_OFF, IN_OFFBOARD, AVOIDING, HOLDING, RETURNING_TO_HOME, LANDING, DISARMING, EXIT};

class MissionState : public State {
	public: 
	MissionState () {};
	MissionState ( MissionStates state ) { this->val = state; cout << "instantiating state and val " << endl; };
	virtual ~MissionState() {};

	// value getter
	MissionStates getValue() {  cout << "getting the value" << endl;  return this->val;};
	void setValue( MissionStates val ) {  cout << "setting the value" << endl;  this->val = val;};
/*

	// value setter
	virtual void setValue(MissionStates val) { this->val = val; };
*/

	private:
	MissionStates val;
};

class MissionStateMachine : public StateMachine
{
	public:
	// constructor
	MissionStateMachine();

	MissionStates getCurrentState()
	{
		return this->current_state.getValue();
	}

	void setCurrentState(MissionStates state )
	{
		this->current_state.setValue(state);
	}


	static string state_map( MissionStates state )
	{

		map<MissionStates, string> state_map_inst;
		state_map_inst[MissionStates::INIT] = "init";
		state_map_inst[MissionStates::CHECKING_PREARM] = "checking_prearm";
		state_map_inst[MissionStates::ARMING] = "arming";
		state_map_inst[MissionStates::TAKING_OFF] = "init";
		state_map_inst[MissionStates::IN_OFFBOARD] = "checking_prearm";
		state_map_inst[MissionStates::AVOIDING] = "arming";
		state_map_inst[MissionStates::HOLDING] = "taking_off";
		state_map_inst[MissionStates::RETURNING_TO_HOME] = "returning to home";
		state_map_inst[MissionStates::LANDING] = "landing";
		state_map_inst[MissionStates::DISARMING] = "disarming";
		state_map_inst[MissionStates::EXIT] = "exit";

		return state_map_inst[state];
	}
	//InOffboardStateMachine in_offboard_sm;

	/*

	// override for determining event type
	virtual bool isAValidEvent(Event* event) { return checkValidEvent<MissionEvent *>( event ); };
	*/

	// main execution and state machine interfacing
	virtual void cycle();

	protected:
	MissionState current_state = MissionState( MissionStates::INIT );
};

