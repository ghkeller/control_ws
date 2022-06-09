
// this is the module responsible for keeping agent sm for collision avoidance

#include "InOffboardStateMachine.h"
#include "Timer.h"

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

// main body of the state machine
void InOffboardStateMachine::cycle()
{

	// prime the next state to be the same as the current state unless changed
	InOffboardStates next_state = this->current_state_ptr()->getValue();

	// check for events
	InOffboardEvents event = this->checkEvents()->getValue();


	//sub state machine for offboard mode
	switch (this->current_state_ptr()->getValue()) {

		case InOffboardStates::INIT:

		if (this->flags.state_entry == true) {
			// state entry execution
			debugOut("	In state 'INIT'...", BLUE);
			this->flags.state_entry = false;
		}

		// STATE TRANSFER CONDITIONS 
		// only cycle through the init state once

		if (true) {
			debugOut("	Next state will be 'SETTING_TARGET'...", CYAN);
			next_state = State::SETTING_TARGET;
			this->flags.state_exit = true;
		}

		if (this->flags.state_exit == true) {
			debugOut("	Exiting 'INIT'...", MAGENTA);
			cout << endl;
			this->flags.state_exit = false;
			this->flags.state_entry = true;
		}

		break;

		case InOffboardStates::SETTING_TARGET:

		if (this->flags.state_entry == true) {
			// state entry execution
			debugOut("	In state 'SETTING_TARGET'...", BLUE);
			this->flags.state_entry = false;

			//set the target here -- we'll otherwise be idle in this state

		}

		// we only needed to set the target, so now we can go into cycling
		if (true) {
			debugOut("	Next state will be 'CYCLING'...", CYAN);
			next_state = State::CYCLING;
			this->flags.state_exit = true;
		}

		if (this->flags.state_exit == true) {
			debugOut("	Exiting 'SETTING_TARGET'...", MAGENTA);
			cout << endl;
			this->flags.state_exit = false;
			this->flags.state_entry = true;
		}

		break;

		case InOffboardStates::CYCLING:

		if (this->flags.state_entry == true) {
			// state entry execution
			debugOut("	In state 'CYCLING'...", BLUE);
			this->flags.state_entry = false;
		}

		// we're cycling the setpoint value currently, so now we need to wait unti we
		// hit the waypoint.

		if (event == Event::WAYPOINT_HIT) {
			debugOut("	The waypoint has been hit.", YELLOW);
			debugOut("	Next state will be 'STALLING_POST_WP_HIT'...", CYAN);
			next_state = State::STALLING_POST_WP_HIT;
			this->flags.state_exit = true;
		}

		if (this->flags.state_exit == true) {
			debugOut("	Exiting 'CYCLING'...", MAGENTA);
			cout << endl;
			this->flags.state_exit = false;
			this->flags.state_entry = true;
		}

		break;

		case InOffboardStates::STALLING_POST_WP_HIT:

		if (this->flags.state_entry == true) {
			// state entry execution
			debugOut("	In state 'STALLING_POST_WP_HIT'...", BLUE);
			this->flags.state_entry = false;

			// this is where to start the stall timer
		}

		// we're cycling the setpoint value currently, so now we need to wait unti we
		// hit the waypoint.

		if (false) {
			debugOut("	The waypoint stall timer has now run out.", YELLOW);
			debugOut("	Next state will be 'SETTING_TARGET'...", CYAN);
			next_state = State::STALLING_POST_WP_HIT;
			this->flags.state_exit = true;
		}

		if (this->flags.state_exit == true) {
			debugOut("	Exiting 'STALLING_POST_WP_HIT'...", MAGENTA);
			cout << endl;
			this->flags.state_exit = false;
			this->flags.state_entry = true;
		}

		break;


		default:

		debugOut("ERROR: dropped into default state -- state machine has lost state!", RED);
		cout << endl;

		break;
	}

	this->current_state_ptr->setValue( next_state );
}
