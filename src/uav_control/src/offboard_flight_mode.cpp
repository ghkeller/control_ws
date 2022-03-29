// offboard_flight_mode.cpp: contains abstract class for a flight mode in offboard

#include <iostream>

#include "state_machine.h"

using namespace std;

namespace OffboardFlightControl
{

    virtual class OffboardFlightMode
    {
        public:
        
        // require a means of providing motion commands
        virtual SetpointInputStream() = 0; 


        private:

        // the specific state machine implemented
        StateMachine stateMachine;

    }

}
