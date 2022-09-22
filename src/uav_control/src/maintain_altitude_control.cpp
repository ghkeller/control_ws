/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
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
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
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

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = hover_height;

    std::vector<std::array<double, 2>> sp_vec = { {{1.0,0.0}}, {{2.0,0.0}} };

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    int sp_it = 0; // indexing variable for waypoints to visit

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
        } else if( current_state.armed && (sp_it < sp_vec.size()) &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            //set the desired position to the next value in our list 
            pose.pose.position.x = sp_vec.at(sp_it)[0];
            pose.pose.position.y = sp_vec.at(sp_it)[1];

            //increment the index
            sp_it++;

            //reset the timer
            last_request = ros::Time::now();
        }
        
        // get current altimeter height
        if (!current_alt_height.ranges.empty()) {
            double height_comm = hover_height + (hover_height - current_alt_height.ranges[0]);
        
            // adjust pose if it is a reasonable adjustment (filter out inf)
            if (height_comm < 10.0 && height_comm > -10.0) {
                pose.pose.position.z = height_comm;
                ROS_INFO("height: %f", pose.pose.position.z);
            }
        }

        // broadcast the new pose desired
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

