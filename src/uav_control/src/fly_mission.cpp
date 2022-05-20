/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL

 * 
 * 
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/PositionTarget.h>
#include <vector>
#include <fstream>
#include <string>
#include <iostream>

#include "PositionTargetScheme.h"
#include "ROSMissionComponents.h"
#include "FlyMissionSM.h"
#include "AgentSM.cpp"

namespace fly_mission
{


bool load_flight(std::string filename) {
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

using namespace fly_mission;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "fly_mission");
    ros::NodeHandle nh;

    ROS_INFO("Starting the 'fly_mission' control node.");

	// read in 
	ROSMissionComponents rmc(nh);

	// load the flight
    ROS_INFO("flight filename: %s", flight_fname.c_str());
    bool succ = rmc->load_flight(flight_fname);
    if (succ)
    {
        ROS_INFO("Successfully loaded flight %s", flight_fname.c_str());
    } else
    {
        ROS_INFO("Did not successfully loaded flight %s", flight_fname.c_str());
    }

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10.0);

    // wait for FCU connection
    ROS_INFO("now waiting for ros to boot up and for the aircraft to be connected");
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("we are connected and ros is up and running. going into offboard...");

    ros::spinOnce();
    rate.sleep();

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    while(ros::ok()){

    	// state machine

    	switch (current_sm_state) {
    		case INIT:
    		{
	    		if (state_entry == true) {
	    			// state entry execution
	    			ROS_INFO("In state 'INIT'...");

	    			state_entry = false;
	    		}


	    		/* STATE TRANSFER CONDITIONS */
	    		// only cycle through the init state once
	    		if (true) {
	    			ROS_INFO("Going to state 'ARMING'...");
	    			next_sm_state = ARMING;
	    			state_entry = true;
	    		}

	    		break;
    		}

    		case ARMING:
    		{
	    		if (state_entry == true) {
	    			// state entry execution
	    			ROS_INFO("In state 'ARMING'...");

	    			// double check that we aren't armed yet
	    			if (!current_state.armed) {
		    			if( arming_client.call(arm_cmd) &&
		                    arm_cmd.response.success){
		                    ROS_INFO("	Vehicle armed.");
		                }

	                    //start our timer
	                    last_request = ros::Time::now();
	            	}

	    			state_entry = false;
	    		}


	    		/* STATE TRANSFER CONDITIONS */
	    		if (ros::Time::now() - last_request > ros::Duration(1.0)) {
	    			// clear our timer
	    			last_request = ros::Time::now();

	    			ROS_INFO("Going to state 'TAKING_OFF'...");
	    			next_sm_state = TAKING_OFF;
	    			state_entry = true;
	    		}

	    		break;
    		}

    		case TAKING_OFF:
    		{
	    		bool takeoff_success;

	    		if (state_entry == true) {
	    			// state entry execution
	    			ROS_INFO("In state 'TAKING_OFF'...");

	    			//takeoff
	            	mavros_msgs::CommandTOL srv_takeoff;
				    srv_takeoff.request.altitude = 2.5; // as of now, doesn't really do anything... it goes off of the takeoff height parameter
				    srv_takeoff.request.latitude = float(NAN);
				    srv_takeoff.request.longitude = float(NAN);
				    srv_takeoff.request.min_pitch = 0;
					srv_takeoff.request.yaw = 0;
					if(takeoff_cl.call(srv_takeoff)){
							takeoff_success = true;
					        ROS_INFO("srv_takeoff send ok %d", srv_takeoff.response.success);
					    }else{
					    	takeoff_success = false;
					        ROS_ERROR("Failed Takeoff");
					}
	    			state_entry = false;
	    		}


	    		/* STATE TRANSFER CONDITIONS */
	    		// change state if we've reached takeoff height
	    		if (abs(current_pose.pose.position.z - 2.5) < wp_dist_thresh) {
	    			// clear our timer
	    			last_request = ros::Time::now();

	    			ROS_INFO("Going to state 'IN_OFFBOARD'...");
	    			next_sm_state = IN_OFFBOARD;
	    			state_entry = true;
	    		}
    		}
    		break;

    		case IN_OFFBOARD:
    		{
	    		if (state_entry == true) {
	    			// state entry execution
	    			ROS_INFO("In state 'IN_OFFBOARD'...");

	    			// tell pixhawk we want to enter offboard mode now
	    			if (current_state.mode != "OFFBOARD") {
	    				if (set_mode_client.call(offb_set_mode) &&
	                		offb_set_mode.response.mode_sent){
	                		ROS_INFO("	Offboard enabled.");
	                	} else {
	                		//throw error
	                		ROS_INFO("	Failed to enter offboard mode.");
	                	}
	    			}

	    			state_entry = false;
	    		}

	    		offboard_sub_sm();

	    		/* STATE TRANSFER CONDITIONS */
	    		if (coll_av_flag == true) {
	    			//clear the flag
	    			coll_av_flag = false;

	    			ROS_INFO("Going to state 'AVOIDING'...");
	    			next_sm_state = AVOIDING;
	    			state_entry = true;
	    		}

	    		break;
	    	}

    		case AVOIDING:
    		{
	    		if (state_entry == true) {
	    			// state entry execution
	    			ROS_INFO("In state 'AVOIDING'...");

	    			agent_sm::init_agent_sm(nh);
	    			state_entry = false;
	    		}

	    		bool finished = agent_sm::agent_sm();

	    		/* STATE TRANSFER CONDITIONS */
	    		if (finished == true) {
	    			ROS_INFO("Going to state 'IN_OFFBOARD'...");
	    			next_sm_state = IN_OFFBOARD;
	    			state_entry = true;
	    		} 

	    		break;
	    	}

    		case HOLDING:
    		{
	    		if (state_entry == true) {
	    			// state entry execution
	    			ROS_INFO("In state 'HOLDING'...");

	    			state_entry = false;
	    		}


	    		/* STATE TRANSFER CONDITIONS */

	    		break;
	    	}

    		default:
    		{
	    		ROS_INFO("Fell through to default case...");

	    		break;
	    	}
    	}

        ros::spinOnce();
        rate.sleep();
 		current_sm_state = next_sm_state;
    }

    return 0;
}
