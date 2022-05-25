#pragma once

/* system includes */
#include <iostream>
#include <string>
#include <cstddef>
#include <queue>

/* local includes */
#include "mavros_msgs/PositionTarget.h"

using namespace std;

class PositionTargetScheme
{
    public:
	/* constructor */
    PositionTargetScheme();

	/* methods */
	int getSetpointQueueSize(void);
	bool addSetpointToQueue(mavros_msgs::PositionTarget);
	bool cycleSPStateMachine();
	mavros_msgs::PositionTarget nextSetpoint();

	/* setters */
	void setName(string name);
	void setIgnoreFlags(uint16_t ignore_flags);
	void setCoordinateFrame(uint16_t coord_frame);
	void setDelayAfterWPHit(uint16_t milliseconds);

	/* getters */
	mavros_msgs::PositionTarget setPositionTarget();
	string getName(void);
	uint16_t getIgnoreFlags(void);
	uint16_t getCoordinateFrame(void);
	uint16_t getDelayAfterWPHit(void);
	bool queueEmpty(void);

    private:
	string name;
	uint16_t ignore_flags;
	uint16_t coord_frame;
	queue<mavros_msgs::PositionTarget> setpoint_queue;

    protected:

};
