// PositionTargetScheme.cpp: source code for interfacing with MAVLink APIs

/* system includes */
#include <iostream>
#include <string>
#include <cstddef>

/* local includes */
#include "PositionTargetScheme.h"
#include "mavros_msgs/PositionTarget.h"

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

void PositionTargetScheme::setIgnoreFlags(uint16_t ignore_flags)
{
	this->ignore_flags = ignore_flags;
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

uint16_t PositionTargetScheme::getIgnoreFlags()
{
	return this->ignore_flags;
}

uint16_t PositionTargetScheme::getCoordinateFrame()
{
	return this->coord_frame;
}


bool PositionTargetScheme::addSetpointToQueue(mavros_msgs::PositionTarget sp)
{
	// no ret value from push, so check queue size to verify push worked 
	uint16_t queue_size_before = this->setpoint_queue.size();
	this->setpoint_queue.push(sp);
	uint16_t queue_size_after = this->setpoint_queue.size();

	if (queue_size_after - queue_size_before != 1) {
		std::cerr << "Queue size did not increment after pushing -- element not added." << std::endl;
		return false;
	}

	return true;
}
