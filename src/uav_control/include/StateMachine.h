#pragma once

#include <iostream>
#include <string>
#include <queue>
#include <vector>

using namespace std;

/* BASE */

class StateMachine
{
	public:
	StateMachine() {};
	virtual ~StateMachine() {};

	class State {
		int value_;
		public:
		virtual ~State () {};
		int & value()				{ return value_; }
		const int & value() const 	{ return value_; }
	};

	class Event {
		int value_;
		public:
		virtual ~Event () {};
		int & value()				{ return value_; }
		const int & value() const 	{ return value_; }
	};

	// primary execution
	virtual void cycle() = 0;

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

	// override this method in each derived state machine, replacing the template param
	virtual bool isAValidEvent( Event* event ) = 0;

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
		for ( StateMachine * sub_state_machine : this->sub_SMs )
		{ 
			sub_state_machine->registerEvents( unprocessed_events );
		}
	}

	// event checker which returns the next event in the FIFO
	Event *checkEvents()
	{ 
		if ( !event_queue.empty() ) {
			Event *next_event = event_queue.front();
			event_queue.pop();
			return next_event;
		} else {
			return nullptr;
		}
	};


	// getters
	int getCurrentState() { return this->current_state_ptr->value(); };
	int eventQueueSize() { return this->event_queue.size(); };
	bool eventQueueEmpty() { return this->event_queue.empty(); };
	
	// setters
	void setCurrentState( int state ) { this->current_state_ptr->value() = state; };

	protected:
	State *current_state_ptr;
	queue<Event *> event_queue;
	vector<StateMachine *> sub_SMs;
};

