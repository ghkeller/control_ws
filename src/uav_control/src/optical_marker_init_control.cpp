/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL

 * This node runs the state machine for the vehicle but does not post mavros position topics
 * (this is left to the thread running in tandem).
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

#include "optical_marker_init_sm.cpp"

namespace optical_marker_init_control
{
//ros specific variables
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
mavros_msgs::PositionTarget current_target;

// definitions
float wp_dist_thresh = 0.25; // meters
float vec_wait_duration = 3; // seconds
int vec_iterator = 0;

//flags
bool coll_av_flag = false;

// use this for timers
ros::Time last_request;
ros::Publisher target_pos_pub;

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	current_pose = *msg;
}

void target_cb(const mavros_msgs::PositionTarget::ConstPtr& msg)
{
    current_target = *msg;
}

void gcs_alert_cb(const std_msgs::String::ConstPtr& msg)
{
	//parse our the message
	std::string recv = msg->data;
    ROS_INFO("Received message from vehicle 0: [%s]", recv.c_str());

	//compare message to cases for flag setting
    if (recv.compare("STOP") == 0) {
    	ROS_INFO("    GCS has commanded STOP.");
    	coll_av_flag = true;
    	return;
    }
}

typedef enum {INIT, ARMING, TAKING_OFF, IN_MARKER_INIT} sm_state;

static sm_state current_sm_state = INIT;
sm_state next_sm_state;
bool state_entry = true;


static offboard_substate current_offboard_substate = SUB_INIT;
offboard_substate next_offboard_substate;
bool offboard_substate_entry = true;

mavros_msgs::PositionTarget desired_target;

//two vectors which store the position targets and their respective times
std::vector<mavros_msgs::PositionTarget> pt_vec;
std::vector<float> vec_time;

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

	        tp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;

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

using namespace uav_node_v2;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_node_v2");
    ros::NodeHandle nh;


    ROS_INFO("Started the uav control node.");

    std::string flight_fname;
    bool avoidance;
    double waypoint_distance_hit_thresh;
    double waypoint_hit_wait_time;
    int starting_waypoint_number;

    nh.getParam(ros::this_node::getNamespace() + "/control/flight_fname",flight_fname);
    nh.getParam(ros::this_node::getNamespace() + "/control/avoidance",avoidance);
    nh.getParam(ros::this_node::getNamespace() + "/control/waypoint_distance_hit_thresh",waypoint_distance_hit_thresh);
    nh.getParam(ros::this_node::getNamespace() + "/control/waypoint_hit_wait_time",waypoint_hit_wait_time);
    nh.getParam(ros::this_node::getNamespace() + "/control/starting_waypoint_number",starting_waypoint_number);

    ROS_INFO("flight filename: %s", flight_fname.c_str());

    bool succ = load_flight(flight_fname);
    if (succ)
    {
        ROS_INFO("Successfully loaded flight %s", flight_fname.c_str());
    } else
    {
        ROS_INFO("Did not successfully loaded flight %s", flight_fname.c_str());
    }


    std::string state_topic = ros::this_node::getNamespace() + "/mavros/state";
    std::string position_topic = ros::this_node::getNamespace() +  "/mavros/local_position/pose";

    std::string setpoint_topic = ros::this_node::getNamespace() + "/mavthread/setpoint_raw/local";
    
    std::string arming_service = ros::this_node::getNamespace() + "/mavros/cmd/arming";
    std::string set_mode_service = ros::this_node::getNamespace() + "/mavros/set_mode";
    std::string takeoff_service = ros::this_node::getNamespace() + "/mavros/cmd/takeoff";

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            (state_topic, 10, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            (position_topic, 10, position_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            (arming_service);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            (set_mode_service);
    ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>
    		(takeoff_service);
    target_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
            (setpoint_topic, 100);
                //declare subscribers
    ros::Subscriber gcs_alert_sub = nh.subscribe<std_msgs::String>
            ("/gcs/vehicle_alert", 10, gcs_alert_cb);

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

    		case IN_MARKER_INIT:
    		{
	    		if (state_entry == true) {
	    			// state entry execution
	    			ROS_INFO("In state 'IN_MARKER_INIT'...");

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

	    		optical_marker_init_control_sm();

	    		/* STATE TRANSFER CONDITIONS */
	    		if (marker_init_done_flag == true) {
	    			//clear the flag
	    			marker_init_done_flag = false;

	    			ROS_INFO("We have initialized to the marker. Now ending flight...");
                    exit_sm();
	    		}

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
