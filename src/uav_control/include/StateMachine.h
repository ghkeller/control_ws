#pragma once

#include <iostream>
#include <queue>
#include <vector>
#include <typeinfo>

using namespace std;

enum class Events { NO_EVENT };

class Event
{
	public:
	Event() {};
	Event( Events val ) { this->val = val; };
	virtual ~Event() {};

	// value getter
	Events getValue() { return this->val; };

	// value setter
	virtual void setValue(Events val) { this->val = val; };

	private:
	Events val;
};

enum class States { INIT };

class State
{
	public:
	State() {};
	State( States val ) { this->val = val; };
	virtual ~State() {};

	// value getter
	States getValue() { cout << "why?" << endl; return this->val; }; 

/*

	// value setter
	virtual void setValue(States val) { this->val = val; };
*/

	private:
	States val;
};

class StateMachine
{
	public:
	StateMachine();

	// primary method : requires implementation in derived classes
	virtual void cycle() = 0;
	
	// method for ingesting the events we need to process
	void registerEvents( queue<Event*>& events_to_register_q );

	// methods for assessing the event type (abstracted through the following method for 
	// ease of overriding
	template<typename EventType>
	bool checkValidEvent(Event* event)
	{
		if ( dynamic_cast<EventType>( event ) != nullptr )
			return true;
		else
			return false;
	}

	// override this method in each derived state Machine, replacing the template param
	virtual bool isAValidEvent( Event* event ) { return checkValidEvent<Event *>( event ); }

/*
	// adds a pointer to a sub state machine for the event processing chain
	void addSubStateMachine( StateMachine* sub_state_machine ) { this->sub_state_machines.push_back( sub_state_machine ); }

	*/
	// get the event at the front of our queue
	Event* checkEvents();

	//getters
	State* getCurrentState(void);


	//setters
	void setCurrentState( State * );

	private:
	vector<StateMachine *> sub_state_machines;

	protected:
	queue<Event*> event_ptr_queue;
	State* current_state_ptr;
	struct transition_flags {
		bool state_entry;
		bool state_exit;
	} flags;

};
