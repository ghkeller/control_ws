// PositionTargetScheme.cpp: source code for interfacing with MAVLink APIs

/* system includes */
#include <iostream>
#include <string>
#include <cstddef>
#include <vector>

/* local includes */
#include "SetpointScheme.h"
#include "mavros_msgs/PositionTarget.h"

// deal with this for our queue (if we need it)
// static int vec_iterator = 0;

using namespace std;

PositionTargetScheme::PositionTargetScheme()
{
    // constructor for the position target method of submitting setpoints
    // for interacting with MAVLink APIs
    std::cout << "Instantiating an instance of the PositionTargetScheme object" << std::endl;
}


void PositionTargetScheme::setName(string name)
{ 
	this->name = name;
	return;
}

void PositionTargetScheme::setIgnoreMask(uint16_t ignore_mask)
{
	this->ignore_mask = ignore_mask;
	return;
}

void PositionTargetScheme::setCoordinateFrame(uint16_t coord_frame)
{
	this->coord_frame = coord_frame;
	return;
}

string PositionTargetScheme::getName()
{
	return this->name;
}

uint16_t PositionTargetScheme::getIgnoreMask()
{
	return this->ignore_mask;
}

uint16_t PositionTargetScheme::getCoordinateFrame()
{
	return this->coord_frame;
}

bool PositionTargetScheme::vecEmpty(void)
{
	return this->setpoint_vec.empty();
}


mavros_msgs::PositionTarget PositionTargetScheme::nextSetpoint(void)
{
	this->setpoint_index++;
	mavros_msgs::PositionTarget next_setpoint;
	if (this->setpoint_index < setpoint_vec.size())
		next_setpoint = this->setpoint_vec[this->setpoint_index];
	return next_setpoint;
}

int PositionTargetScheme::getSetpointVecSize(void)
{
	return this->setpoint_vec.size();
}


bool PositionTargetScheme::addSetpointToVec(mavros_msgs::PositionTarget sp)
{
	// no ret value from push, so check queue size to verify push worked 
	uint16_t vec_size_before = this->setpoint_vec.size();
	this->setpoint_vec.push_back(sp);
	uint16_t vec_size_after = this->setpoint_vec.size();

	if (vec_size_after - vec_size_before != 1) {
		std::cerr << "Vector size did not increment after pushing -- element not added." << std::endl;
		return false;
	}

	return true;
}
