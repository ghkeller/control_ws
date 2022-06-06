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
	virtual ~Event() {};
	Event( Events val ) { this->val = val; };
	Events val;
};


class StateMachine
{
	public:
	StateMachine() {};
	virtual ~StateMachine() {};
	
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

	// method for ingesting the events we need to process
	void registerEvents( queue<Event*>& events_to_register_q )
	{
		// add events to this state machine to be processed
		queue<Event*> unprocessed_events;
		while ( !events_to_register_q.empty() )
		{
			cout << "processing an event..." << endl;
			Event* event_ptr = events_to_register_q.front();
			if ( this->isAValidEvent( event_ptr ) ) {
				cout << "	... adding this event to the locally maintained queue." << endl;
				this->event_queue.push( event_ptr );
			} else {
				cout << "	...event is not valid for this sm. passing to substates..." << endl;
				unprocessed_events.push( event_ptr );
			}
			events_to_register_q.pop();
		}


		// pass unprocessed events to the substate machines
		for ( StateMachine * sub_state_machine : this->sub_state_machines )
		{ 
			sub_state_machine->registerEvents( unprocessed_events );
		}
	}

	void addSubStateMachine( StateMachine* sub_state_machine ) { this->sub_state_machines.push_back( sub_state_machine ); }

	private:

	vector<StateMachine *> sub_state_machines;
	queue<Event*> event_queue;
};
