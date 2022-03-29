
// this is the module responsible for keeping agent sm for collision avoidance
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

namespace marker_init_sm
{
//state definitions and instantiations
typedef enum states{STOPPING, WAITING_FOR_ID, WAITING_FOR_WPS, CIRCUMNAVIGATING, HALTING} state;
static state curr_state = STOPPING;
state next_state = curr_state;

//substate defines
typedef enum {SUB_INIT, SETTING_TARGET, CYCLING} avoid_substate;
static avoid_substate current_avoid_substate = SUB_INIT;
avoid_substate next_avoid_substate;
bool avoid_substate_entry = true;

//flags
static bool state_entry_flag = true;
static bool halt_recv = false;
static bool wps_recv = false;
static bool pt_reached_flag = false;

//other
typedef enum IDS{NONE, STOPPER, GOER} coll_av_id;
static coll_av_id id = NONE;
float wp_visit_timeout = 10.0; //seconds
float wp_dist_thresh = 0.2; //m

//pose declarations
geometry_msgs::PoseStamped desired_target;
geometry_msgs::PoseStamped uav_pose;

//pubs/subs
ros::Publisher vehicle_ack_pub;
ros::Publisher target_pos_pub;
ros::Subscriber gcs_msg_sub;
ros::Subscriber gcs_wps_sub;
ros::Subscriber uav_pos_sub;
ros::Subscriber uav_glob_pos_sub;

// use this for timers
ros::Time last_request;

// typedef struct timed_pts {
// 	mavros_msgs::PositionTarget desired_target;
// 	float time;
// } timed_pt;

// std::vector<timed_pts> pt_vec;
std::vector<geometry_msgs::PoseStamped> pt_vec;

bool pt_reached() {
	float x_diff = abs(uav_pose.pose.position.x - desired_target.pose.position.x);
	float y_diff = abs(uav_pose.pose.position.y - desired_target.pose.position.y);
	float z_diff = abs(uav_pose.pose.position.z - desired_target.pose.position.z);
	ROS_INFO("DISTANCES TO X Y Z %f %f %f", x_diff, y_diff, z_diff);
	if (abs(uav_pose.pose.position.x - desired_target.pose.position.x) < wp_dist_thresh &&
		abs(uav_pose.pose.position.y - desired_target.pose.position.y) < wp_dist_thresh &&
		abs(uav_pose.pose.position.z - desired_target.pose.position.z) < wp_dist_thresh) {
		ROS_INFO("Reached waypoint, moving onto next one");
		return true;
	} else {
		return false;
	}
}

void vehicle_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
	//update the recorded position of vehicle 0
	uav_pose.pose.position.x = msg->pose.position.x;
	uav_pose.pose.position.y = msg->pose.position.y;
	uav_pose.pose.position.z = msg->pose.position.z;

	std::stringstream ss;
	ss << std::endl << msg->pose.position.x << std::endl;
	ss << msg->pose.position.y << std::endl;
	ss << msg->pose.position.z << std::endl;
    ROS_INFO("Vehicle 0 position: [%s]", ss.str().c_str());
}

void gcs_msg_cb(const std_msgs::String::ConstPtr& msg)
{
	//parse our the message
	std::string recv = msg->data;
    ROS_INFO("Received message from vehicle 0: [%s]", recv.c_str());

    //compare message to cases for flag setting
    if (recv.compare("STOPPER") == 0) {
    	ROS_INFO("    GSC has assigned us the STOPPER.");
    	id = STOPPER;
    } else if (recv.compare("GOER") == 0) {
    	ROS_INFO("    GSC has assigned us the GOER.");
    	id = GOER;
    } else {

    }
}

void gcs_wps_cb(const std_msgs::String::ConstPtr& msg)
{
	//parse our the message
	std::string recv = msg->data;
    ROS_INFO("Received message from vehicle 0: [%s]", recv.c_str());


    //save the waypoints
    std::stringstream pt_string;
    pt_string << recv;
    std::string pt_string_item;
    for (int i = 0; i < 4; i++){

        geometry_msgs::PoseStamped tp;

        std::getline(pt_string,pt_string_item,',');
        tp.pose.position.x = stof(pt_string_item); //temporary
        std::getline(pt_string,pt_string_item,',');
        tp.pose.position.y = stof(pt_string_item); //temporary
        tp.pose.position.z = 5.0; //m

	    pt_vec.push_back(tp);
    }

    ROS_INFO("Stored avoidance waypoints");

    //set flag high that we have recieved waypoints
    wps_recv = true;
}

bool avoid_sub_sm(std::vector<geometry_msgs::PoseStamped>& pt_vec) {
	static int vec_iterator = 0;

	//sub state machine for offboard mode
	switch (current_avoid_substate) {
		case SUB_INIT:

		if (avoid_substate_entry == true) {
			// state entry execution
			ROS_INFO("In state 'SUB_INIT'...");

			avoid_substate_entry = false;
		}


		/* STATE TRANSFER CONDITIONS */
		// only cycle through the init state once
		if (true) {
			ROS_INFO("Going to state 'SETTING_TARGET'...");
			next_avoid_substate = SETTING_TARGET;
			avoid_substate_entry = true;
		}

		break;

		case SETTING_TARGET:

		if (avoid_substate_entry == true) {
			// state entry execution
			ROS_INFO("In state 'SETTING_TARGET'...");

			if (vec_iterator < pt_vec.size()) {
				// get next waypoint to traverse to
				ROS_INFO("	Getting vector %d", vec_iterator);
				desired_target = pt_vec.at(vec_iterator);

			    ROS_INFO("  position.x: %f", desired_target.pose.position.x);
			    ROS_INFO("  position.y: %f", desired_target.pose.position.y);
			    ROS_INFO("  position.z: %f", desired_target.pose.position.z);

			    ROS_INFO("...redirecting to mavros...");

				vec_iterator++; 
			}
            //start timer
            last_request = ros::Time::now();

			avoid_substate_entry = false;
		}

        target_pos_pub.publish(desired_target);
        ros::spinOnce();


		/* STATE TRANSFER CONDITIONS */
		 //we only want to send the target value once to the handling thread
		 if (true) {
		 	ROS_INFO("Going to state 'CYCLING'...");
		 	next_avoid_substate = CYCLING;
		 	avoid_substate_entry = true;
		 }

		break;

		case CYCLING:

		if (avoid_substate_entry == true) {
			// state entry execution
			ROS_INFO("In state 'CYCLING'...");
            //start timer
            last_request = ros::Time::now();

			avoid_substate_entry = false;
		}

		pt_reached_flag = pt_reached();

		/* STATE TRANSFER CONDITIONS */
		// when we've reached the waypoint, we should transfer to setting or leaving
		if (pt_reached_flag == true && vec_iterator > 3) {
			ROS_INFO("Finished traversing alternate waypoints");
			pt_reached_flag = false;
			return true; // indicating that we are finished traversing
		} else if (pt_reached_flag == true && vec_iterator <= 3) {
			ROS_INFO("Going to state 'SETTING_TARGET'...");
			pt_reached_flag = false;
			next_avoid_substate = SETTING_TARGET;
			avoid_substate_entry = true;
		}

		//if it is taking unreasonably long to get to the next waypoint, assume we are close but toilet bowling
		if (ros::Time::now() - last_request > ros::Duration(wp_visit_timeout)) {
			ROS_INFO("Going to state 'SETTING_TARGET'...");
			next_avoid_substate = SETTING_TARGET;
			avoid_substate_entry = true;
		}

		break;

		default:

		break;
	}

	current_avoid_substate = next_avoid_substate;
	return false; // indicating that we are not finished with our alternate waypoints
}

void init_agent_sm(ros::NodeHandle nh)
{
	//configure ros node with new pubs, subs, etc.
	//declare publishers
	vehicle_ack_pub = nh.advertise<std_msgs::String>("/message", 100);
    target_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavthread/setpoint_position/local", 100);

	//declare subscribers
    gcs_msg_sub = nh.subscribe<std_msgs::String>
            ("gcs/vehicle_id", 10, agent_sm::gcs_msg_cb);
    gcs_wps_sub = nh.subscribe<std_msgs::String>
            ("gcs/goer_wps", 100, agent_sm::gcs_wps_cb);
    uav_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
    		("/mavros/local_position/pose", 100, agent_sm::vehicle_position_cb);
   	
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    //sit and spin for a second before entering state machine
    for (int i = 0; i < 10; i++) {
    	ros::spinOnce();
        rate.sleep();
    }

}

bool agent_sm()
{
//drop into state
	switch(curr_state) {
		case STOPPING:
		{
			//tell the vehicles to stop
			if (state_entry_flag == true) {
				ROS_INFO("------------------------v");
				ROS_INFO("In state 'STOPPING'...");
				ROS_INFO("    -> Entry function...");

				//set the current pose as the setpoint for the vehicles

				

				//make sure we don't come back in
				state_entry_flag = false;
				ROS_INFO("    -> Exiting entry function...");
			}

			//condition to leave state
			if (true) {
				//acknowledge that we have stopped
				std_msgs::String msg;
	    		msg.data = "HALT_ACK";
				vehicle_ack_pub.publish(msg);

				//enter the next state
				state_entry_flag = true;
				next_state = WAITING_FOR_ID;
				ROS_INFO("Leaving state 'STOPPING', going to 'WAITING_FOR_ID'...");
				ROS_INFO("------------------------^");
			}
		}
		break;

		case WAITING_FOR_ID:
		{
			//tell the vehicles to stop
			if (state_entry_flag == true) {
				ROS_INFO("------------------------v");
				ROS_INFO("In state 'WAITING_FOR_ID'...");
				ROS_INFO("    -> Entry function...");

				//make sure we don't come back in
				state_entry_flag = false;
				ROS_INFO("    -> Exiting entry function...");
			}

			//do every cycle we are in this state

			if (id == STOPPER) {

				//acknowledge that we have stopped
				std_msgs::String msg;
	    		msg.data = "ID_ACK";
				vehicle_ack_pub.publish(msg);

				//enter the next state
				next_state = HALTING;
				ROS_INFO("Leaving state 'WAITING_FOR_ID', going to 'HALTING'...");
				ROS_INFO("------------------------^");
				state_entry_flag = true;
			}

			if (id == GOER) {

				//acknowledge that we have stopped
				std_msgs::String msg;
	    		msg.data = "ID_ACK";
				vehicle_ack_pub.publish(msg);

				//enter the next state
				state_entry_flag = true;
				next_state = WAITING_FOR_WPS;
				ROS_INFO("Leaving state 'WAITING_FOR_ID', going to 'WAITING_FOR_WPS'...");
				ROS_INFO("------------------------^");
				state_entry_flag = true;
			}
		}
		break;

		case WAITING_FOR_WPS:
		{
			//tell the vehicles to stop
			if (state_entry_flag == true) {
				ROS_INFO("------------------------v");
				ROS_INFO("In state 'WAITING_FOR_WPS'...");
				ROS_INFO("    -> Entry function...");

				//make sure we don't come back in
				state_entry_flag = false;
				ROS_INFO("    -> Exiting entry function...");
			}

			//do something every cycle here

			if (wps_recv == true) {
				//enter the next state
				next_state = CIRCUMNAVIGATING;
				ROS_INFO("Leaving state 'WAITING_FOR_WPS', going to 'CIRCUMNAVIGATING'...");
				ROS_INFO("------------------------^");
				state_entry_flag = true;
			}

		}
		break;

		case CIRCUMNAVIGATING:
		{
			//tell the vehicles to stop
			if (state_entry_flag == true) {
				ROS_INFO("------------------------v");
				ROS_INFO("In state 'CIRCUMNAVIGATING'...");
				ROS_INFO("    -> Entry function...");

				//make sure we don't come back in
				state_entry_flag = false;
				ROS_INFO("    -> Exiting entry function...");
			}

			//do something every cycle here
			bool finished = avoid_sub_sm(pt_vec);

			if (finished == true) {
				//exit the state machine after posting notice of finishing circumnavigation
				std_msgs::String msg;
	    		msg.data = "DONE_AVOIDNG";
				vehicle_ack_pub.publish(msg);
				ros::spinOnce();
				ROS_INFO("Leaving avoidance subroutine");
				ROS_INFO("------------------------^");
				return true;
			}

		}
		break;


		default:

		break;
	}

	curr_state = next_state;
    ros::spinOnce();
}
}
