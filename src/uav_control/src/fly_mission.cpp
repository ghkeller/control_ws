/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL

 * 
 * 
 */

//* system includes *//
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

//* local includes *//
#include "PositionTargetScheme.h"
#include "ROSMissionComponents.h"
#include "FlyMissionSM.h"
#include "AgentSM.h"
#include "Flight.h"


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
