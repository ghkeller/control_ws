#pragma once

/* system includes */
#include <iostream>
#include <string>
#include <cstddef>
#include <vector>

/* local includes */
#include "mavros_msgs/PositionTarget.h"

using namespace std;

class PositionTargetScheme
{
    public:
	/* constructor */
    PositionTargetScheme();

	/* methods */
	int setpointVecSize(void);
	bool addSetpointToVec(mavros_msgs::PositionTarget);
	int getSetpointVecSize();
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
	bool vecEmpty(void);

    private:
	string name;
	uint16_t ignore_mask;
	uint16_t coord_frame;
	vector<mavros_msgs::PositionTarget> setpoint_vec;
	int setpoint_index = -1; // since we start with 'next sepoint' method

    protected:

};
