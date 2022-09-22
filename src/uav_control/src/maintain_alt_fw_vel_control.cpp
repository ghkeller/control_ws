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

mavros_msgs::State current_state;
sensor_msgs::LaserScan current_alt_height;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void alt_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
    current_alt_height = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber alt_sub = nh.subscribe<sensor_msgs::LaserScan>
            ("laser_altimeter/laser/scan", 10, alt_cb);
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

    // Terr. following PID vars/terms
    double e_sum = 0;
    double e_last = 0;
    double KP = 1.0;
    double KI = 0.0;
    double KD = 0.0;

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
            (ros::Time::now() - last_request > ros::Duration(15.0))){
            //
            //set the forward velocity/update the ignore flags
            pt.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                mavros_msgs::PositionTarget::IGNORE_PY | 
                mavros_msgs::PositionTarget::IGNORE_PZ | 
                mavros_msgs::PositionTarget::IGNORE_AFX | 
                mavros_msgs::PositionTarget::IGNORE_AFY | 
                mavros_msgs::PositionTarget::IGNORE_AFZ | 
                mavros_msgs::PositionTarget::IGNORE_YAW_RATE; 
            pt.velocity.x = 0.5;
            pt.velocity.y = 0.0;
            pt.velocity.z = 0.0;

            //reset the timer
            last_request = ros::Time::now();
        }
        
        // get current altimeter height
        if (!current_alt_height.ranges.empty()) {
            // err
            double e = hover_height - current_alt_height.ranges[0];
            
            // err deriv.
            double e_delta = e - e_last;
            
            // anti-windup conditional/err int.
            if (e_sum < 10.0 && e_sum > -10.0)
                e_sum = e_sum + e;
        
            // adjust pose if it is a reasonable adjustment (filter out inf)
            if (e < 10.0 && e > -10.0) {
                
                // control law
                pt.velocity.z = KP*e + KI*e_sum + KI*e_delta;
                ROS_INFO("height: %f", pt.position.z);
            }
        }

        // broadcast the new position target desired
        local_pos_targ_pub.publish(pt);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

