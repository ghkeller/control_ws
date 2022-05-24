// FlightParsing.cpp: source code for handling flight file parsing 

#include <string>
#include <cstddef>
#include <iostream>

#include "Flight.h"
#include "Parsing.h"
#include "SetpointScheme.h"

// constructor
static void FlightParsing::flightFromCsv(std::string _path_to_CSV, PositionTargetScheme& pts)
{
	std::ifstream data(_path_to_CSV);
    std::string line;
    while(std::getline(data,line)) {
    	try {
	        std::stringstream lineStream(line);
	        std::string cell;
	        mavros_msgs::PositionTarget tp;

	        tp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

	        //px
	        std::getline(lineStream,cell,',');
	        if (!cell.empty()) {
	        	tp.position.x = std::stof(cell);
	        	ROS_INFO("Parsed px");
	        } else {
	        	tp.position.x = 0.0f;
	        	tp.type_mask |= mavros_msgs::PositionTarget::IGNORE_PX;
	        }

	        //py
	        std::getline(lineStream,cell,',');
	        if (!cell.empty()) {
	        	tp.position.y = std::stof(cell);
	        	ROS_INFO("Parsed py");
	        } else {
	        	tp.position.y = 0.0f;
	        	tp.type_mask |= mavros_msgs::PositionTarget::IGNORE_PY;
	        }

	        //pz
	        std::getline(lineStream,cell,',');
	        if (!cell.empty()) {
	        	tp.position.z = std::stof(cell);
	        	ROS_INFO("Parsed pz");
	        } else {
	        	tp.position.z = 0.0f;
	        	tp.type_mask |= mavros_msgs::PositionTarget::IGNORE_PZ;
	        }

	        //ax
	        std::getline(lineStream,cell,',');
	        if (!cell.empty()) {
	        	tp.acceleration_or_force.x = std::stof(cell);
	        	ROS_INFO("Parsed ax");
	        } else {
	        	tp.acceleration_or_force.x = 0.0f;
	        	tp.type_mask |= mavros_msgs::PositionTarget::IGNORE_AFX;
	        }

	        //ay
	        std::getline(lineStream,cell,',');
	        if (!cell.empty()) {
	        	tp.acceleration_or_force.y = std::stof(cell);
	        	ROS_INFO("Parsed ay");
	        } else {
	        	tp.acceleration_or_force.y = 0.0f;
	        	tp.type_mask |= mavros_msgs::PositionTarget::IGNORE_AFY;
	        }

	        //az
	        std::getline(lineStream,cell,',');
	        if (!cell.empty()) {
	        	tp.acceleration_or_force.z = std::stof(cell);
	        	ROS_INFO("Parsed az");
	        } else {
	        	tp.acceleration_or_force.z = 0.0f;
	        	tp.type_mask |= mavros_msgs::PositionTarget::IGNORE_AFZ;
	        }

	       	//vx
	        std::getline(lineStream,cell,',');
	        if (!cell.empty()) {
	        	tp.velocity.x = std::stof(cell);
	        	ROS_INFO("Parsed vx");
	        } else {
	        	tp.velocity.x = 0.0f;
	        	tp.type_mask |= mavros_msgs::PositionTarget::IGNORE_VX;
	        }

	        //vy
	        std::getline(lineStream,cell,',');
	        if (!cell.empty()) {
	        	tp.velocity.y = std::stof(cell);
	        	ROS_INFO("Parsed vy");
	        } else {
	        	tp.velocity.y = 0.0f;
	        	tp.type_mask |= mavros_msgs::PositionTarget::IGNORE_VY;
	        }

	        //vz
	        std::getline(lineStream,cell,',');
	        if (!cell.empty()) {
	        	tp.velocity.z = std::stof(cell);
	        	ROS_INFO("Parsed vz");
	        } else {
	        	tp.velocity.z = 0.0f;
	        	tp.type_mask |= mavros_msgs::PositionTarget::IGNORE_VZ;
	        }

	        //yaw
	        std::getline(lineStream,cell,',');
	        if (!cell.empty()) {
	        	tp.yaw = std::stof(cell);
	        	ROS_INFO("Parsed yaw");
	        } else {
	        	tp.yaw = 0.0f;
	        	tp.type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW;
	        }

	        //yaw
	        std::getline(lineStream,cell,',');
	        if (!cell.empty()) {
	        	tp.yaw_rate = std::stof(cell);
	        	ROS_INFO("Parsed yaw rate");
	        } else {
	        	tp.yaw_rate = 0.0f;
	        	tp.type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
	        }

	        //save the position target struct to our vector
    		pts->addSetpointToQueue(tp);
    		ROS_INFO("Added position target to vector");
	    }

	    catch (...)  { 
        	ROS_INFO("Error parsing line of csv. Moving on to next line...");
    	} 
    }

    ROS_INFO("Finished parsing csv file.");
    return true;
	return;
}

