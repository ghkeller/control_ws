/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <cmath>

mavros_msgs::State current_state;
double current_slam_height = 0.0;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void height_cb(const std_msgs::Float32::ConstPtr& msg){
    current_slam_height = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber height_sub = nh.subscribe<std_msgs::Float32>
            ("/slam/local_height_est", 10, height_cb);
    ros::Publisher local_pos_targ_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    double hover_height = 2.0;

    mavros_msgs::PositionTarget pt;
    pt.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    pt.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
        mavros_msgs::PositionTarget::IGNORE_VY | 
        mavros_msgs::PositionTarget::IGNORE_VZ | 
        mavros_msgs::PositionTarget::IGNORE_AFX | 
        mavros_msgs::PositionTarget::IGNORE_AFY | 
        mavros_msgs::PositionTarget::IGNORE_AFZ | 
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE; 
    pt.position.x = 0;
    pt.position.y = 0;
    pt.position.z = hover_height;
    pt.yaw = 0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_targ_pub.publish(pt);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();


    // apply the direction angle to the lateral vel
    double yaw = -0.827;
    double horiz_speed = 0.5; // m/s
    double x_vel = horiz_speed * cos(yaw);
    double y_vel = horiz_speed * sin(yaw);


    // Terr. following PID vars/default terms
    double e_sum = 0;
    double e_last = 0;
    double KP = 1.0;
    double KI = 0.0;
    double KD = 0.0;

    // read in params
    nh.getParam("/control/terr_follow_KP", KP);
    nh.getParam("/control/terr_follow_KI", KI);
    nh.getParam("/control/terr_follow_KD", KD);

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else if ( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
        } else if( current_state.armed && 
            (ros::Time::now() - last_request > ros::Duration(15.0)) &&
            (ros::Time::now() - last_request <= ros::Duration(20.0))){
            //
            //
	    //go to the desired start point
            pt.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                mavros_msgs::PositionTarget::IGNORE_VY | 
                mavros_msgs::PositionTarget::IGNORE_VZ | 
                mavros_msgs::PositionTarget::IGNORE_AFX | 
                mavros_msgs::PositionTarget::IGNORE_AFY | 
                mavros_msgs::PositionTarget::IGNORE_AFZ | 
                mavros_msgs::PositionTarget::IGNORE_YAW_RATE; 
            pt.position.x = 0.0;
            pt.position.y = 0.0;
            pt.position.z = 10.0;
		ROS_INFO("Going to the takeoff point...");

        } else if( current_state.armed && 
	    (ros::Time::now() - last_request > ros::Duration(20.0)) &&
            (ros::Time::now() - last_request <= ros::Duration(110.0))){
            //
            //
	    //go to the desired start point
            pt.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                mavros_msgs::PositionTarget::IGNORE_VY | 
                mavros_msgs::PositionTarget::IGNORE_VZ | 
                mavros_msgs::PositionTarget::IGNORE_AFX | 
                mavros_msgs::PositionTarget::IGNORE_AFY | 
                mavros_msgs::PositionTarget::IGNORE_AFZ | 
                mavros_msgs::PositionTarget::IGNORE_YAW_RATE; 
            pt.position.x = 76.6;
            pt.position.y = -146.9;
            pt.position.z = 30.8;
            pt.yaw = yaw;
		ROS_INFO("Going to the start point for tf...");

        } else if( current_state.armed && 
	    (ros::Time::now() - last_request > ros::Duration(110.0))){
		//
		//
		// conduct the terrain following
		//

		// apply to the velocity command
		pt.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
			mavros_msgs::PositionTarget::IGNORE_PY | 
			mavros_msgs::PositionTarget::IGNORE_PZ | 
			mavros_msgs::PositionTarget::IGNORE_AFX | 
			mavros_msgs::PositionTarget::IGNORE_AFY | 
			mavros_msgs::PositionTarget::IGNORE_AFZ | 
			mavros_msgs::PositionTarget::IGNORE_YAW_RATE; 
		pt.velocity.x = x_vel;
		pt.velocity.y = y_vel;
		pt.velocity.z = 0.0;

		// listen for the height difference and apply to fw vel command

		if (current_slam_height > 0.0001) {
		    // err
		    double e = hover_height - current_slam_height;
		    
		    // err deriv.
		    double e_delta = e - e_last;
		    e_last = e;
		    
		    // anti-windup conditional/err int.
		    if (e_sum < 10.0 && e_sum > -10.0)
			e_sum = e_sum + e;
		
		    // adjust pose if it is a reasonable adjustment (filter out inf)
		    if (e < 10.0 && e > -10.0) {
			
			// control law
			pt.velocity.z = KP*e + KI*e_sum + KD*e_delta;
			ROS_INFO("height: %f", pt.position.z);
		    }
		}

        }


        // broadcast the new position target desired
        local_pos_targ_pub.publish(pt);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

