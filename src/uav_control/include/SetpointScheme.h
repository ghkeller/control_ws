#pragma once

/* system includes */
#include <iostream>
#include <string>
#include <cstddef>
#include <list>

/* local includes */
#include "mavros_msgs/PositionTarget.h"

using namespace std;

class PositionTargetScheme
{
    public:
	/* constructor */
    PositionTargetScheme();

	/* methods */
	int setpointListSize(void);
	bool addSetpointToList(mavros_msgs::PositionTarget);
	mavros_msgs::PositionTarget nextSetpoint();

	/* setters */
	void setName(string name);
	void setIgnoreMask(uint16_t ignore_flags);
	void setCoordinateFrame(uint16_t coord_frame);
	void setDelayAfterWPHit(uint16_t milliseconds);

	/* getters */
	string getName(void);
	uint16_t getIgnoreMask(void);
	uint16_t getCoordinateFrame(void);
	bool listEmpty(void);

    private:
	string name;
	uint16_t ignore_flags;
	uint16_t coord_frame;
	list<mavros_msgs::PositionTarget> setpoint_list;
	int setpoint_index;

    protected:

};
