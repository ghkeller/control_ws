// FlightParsing.cpp: source code for handling flight file parsing 

#include <string>
#include <cstddef>
#include <iostream>
#include <fstream>

#include "Flight.h"
#include "Parsing.h"
#include "SetpointScheme.h"
#include "mavros_msgs/PositionTarget.h"


using namespace std;

// static parsing func
bool Parsing::flightFromCsv(string _path_to_CSV, PositionTargetScheme& pts)
{
	bool success_flag = true;

	// prepare data stream to parse
	ifstream data(_path_to_CSV);

	// check to see if the flight file exists
	if(!data.is_open()){
		std::cout << "File not found at path: " << _path_to_CSV << std::endl;
		return false;
	}

	// parse by line in the file, starting on the second line
    string line;
	getline(data,line); // here's where we throw away the first line
    while(getline(data,line)) {
		// should quantify the number of cells to verify that the file was created
		// properly prior to parsing -- do this later.
    	try {
	        stringstream lineStream(line);
	        string cell;
	        mavros_msgs::PositionTarget tp;

	        tp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

	        //px
	        getline(lineStream,cell,',');
	        if (!cell.empty()) {
	        	tp.position.x = stof(cell);
	        	cout << "Parsed px" << endl;

	        } else {
	        	tp.position.x = 0.0f;
	        	tp.type_mask |= mavros_msgs::PositionTarget::IGNORE_PX;
	        }

	        //py
	        getline(lineStream,cell,',');
	        if (!cell.empty()) {
	        	tp.position.y = stof(cell);
	        	cout << "Parsed py" << endl;
	        } else {
	        	tp.position.y = 0.0f;
	        	tp.type_mask |= mavros_msgs::PositionTarget::IGNORE_PY;
	        }

	        //pz
	        getline(lineStream,cell,',');
	        if (!cell.empty()) {
	        	tp.position.z = stof(cell);
	        	cout << "Parsed pz" << endl;
	        } else {
	        	tp.position.z = 0.0f;
	        	tp.type_mask |= mavros_msgs::PositionTarget::IGNORE_PZ;
	        }

	        //ax
	        getline(lineStream,cell,',');
	        if (!cell.empty()) {
	        	tp.acceleration_or_force.x = stof(cell);
	        	cout << "Parsed ax" << endl;
	        } else {
	        	tp.acceleration_or_force.x = 0.0f;
	        	tp.type_mask |= mavros_msgs::PositionTarget::IGNORE_AFX;
	        }

	        //ay
	        getline(lineStream,cell,',');
	        if (!cell.empty()) {
	        	tp.acceleration_or_force.y = stof(cell);
	        	cout << "Parsed ay" << endl;
	        } else {
	        	tp.acceleration_or_force.y = 0.0f;
	        	tp.type_mask |= mavros_msgs::PositionTarget::IGNORE_AFY;
	        }

	        //az
	        getline(lineStream,cell,',');
	        if (!cell.empty()) {
	        	tp.acceleration_or_force.z = stof(cell);
	        	cout << "Parsed az" << endl;
	        } else {
	        	tp.acceleration_or_force.z = 0.0f;
	        	tp.type_mask |= mavros_msgs::PositionTarget::IGNORE_AFZ;
	        }

	       	//vx
	        getline(lineStream,cell,',');
	        if (!cell.empty()) {
	        	tp.velocity.x = stof(cell);
	        	cout << "Parsed vx" << endl;
	        } else {
	        	tp.velocity.x = 0.0f;
	        	tp.type_mask |= mavros_msgs::PositionTarget::IGNORE_VX;
	        }

	        //vy
	        getline(lineStream,cell,',');
	        if (!cell.empty()) {
	        	tp.velocity.y = stof(cell);
	        	cout << "Parsed vy" << endl;
	        } else {
	        	tp.velocity.y = 0.0f;
	        	tp.type_mask |= mavros_msgs::PositionTarget::IGNORE_VY;
	        }

	        //vz
	        getline(lineStream,cell,',');
	        if (!cell.empty()) {
	        	tp.velocity.z = stof(cell);
	        	cout << "Parsed vz" << endl;
	        } else {
	        	tp.velocity.z = 0.0f;
	        	tp.type_mask |= mavros_msgs::PositionTarget::IGNORE_VZ;
	        }

	        //yaw
	        getline(lineStream,cell,',');
	        if (!cell.empty()) {
	        	tp.yaw = stof(cell);
	        	cout << "Parsed yaw" << endl;
	        } else {
	        	tp.yaw = 0.0f;
	        	tp.type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW;
	        }

	        //yaw
	        getline(lineStream,cell,',');
	        if (!cell.empty()) {
	        	tp.yaw_rate = stof(cell);
	        	cout << "Parsed yaw rate" << endl;
	        } else {
	        	tp.yaw_rate = 0.0f;
	        	tp.type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
	        }

	        //save the position target struct to our vector
    		pts.addSetpointToQueue(tp);
    		cout << "Added position target to vector" << endl;
	    }

	    catch (...)  { 
        	cout << "Error parsing line of csv. Moving on to next line. .."<< endl;
			success_flag = false;
    	} 
    }

    cout << "Finished parsing csv file. "<< endl;

	return success_flag; // need to return a failure or success
}

