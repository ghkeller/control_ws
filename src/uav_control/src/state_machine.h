#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_


#include <iostream>

class StateMachine
{
    public:
    void cycle(void);
    State getCurrentState();
    State getNextStates();
    
    private:

    State _initial_state;
    State _current_state;

}

class Transition
{
    public:
    State getFromState();
    State getToState();
    
    private:
    State _from_state;
    State _to_state;

}

class Event
{
    public:
    
    private:
}

class State
{
    public:

    void State(string name)
    {
        this._name = name;
    }

    string getStateName(void)
    {
        return this._name;
    }

    int entry(State from_state)
    {
        // functionality to execute on entry
        // returns 0 on successful execution or nonzero for unsuccessful
        stdout << "Entering " << this.getStateName() << "...";

    }

    int exit(State to_state)
    {
        // functionality to execute on exit
        // returns 0 on successful execution or nonzero for unsuccessful
        stdout << "Exiting state " << this.getStateName() << "...";

    }

    private:

    string _name;

}

#endif //STATEMACHINE_H_
