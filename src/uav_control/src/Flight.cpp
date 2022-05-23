// Flight.cpp: source code for handling flight planning

#include <string>
#include <cstddef>
#include <iostream>

#include "Flight.h"

namespace fly_mission
{

// constructor
Flight::Flight(std::string filename)
{

}

// parse out a flight from a file
bool Flight::load_flight(std::string filename) {
	//load the csv values
    std::string full_flight_fname_path = ros::package::getPath("uav_control") + "/flights/" + filename + ".csv";
    ROS_INFO(full_flight_fname_path.c_str());
	std::ifstream data(full_flight_fname_path);
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

	        //time
	        std::getline(lineStream,cell,',');
	        if (!cell.empty()) {
	        	vec_time.push_back(std::stof(cell));
	        	ROS_INFO("Parsed time");
	        }

	        //save the position target struct to our vector
    		pt_vec.push_back(tp);
    		ROS_INFO("Added position target to vector");
	    }

	    catch (...)  { 
        	ROS_INFO("Error parsing line of csv. Moving on to next line...");
    	} 
    }

    ROS_INFO("Finished parsing csv file.");
    return true;
}
}
