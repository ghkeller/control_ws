/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL

 * This thread is dedicated to posting the offboard positions (as suggested by
 * the Mavros documentation)
 * 
 */

#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <vector>

ros::Publisher target_pos_pub;
mavros_msgs::PositionTarget tp;

void target_cb(const mavros_msgs::PositionTarget::ConstPtr& msg)
{
    ROS_INFO("Recieved target position message: ");
    ROS_INFO("  coordinate frame: %d", msg->coordinate_frame);
    ROS_INFO("  type mask: %d", msg->type_mask);
    ROS_INFO("  position.x: %f", msg->position.x);
    ROS_INFO("  position.y: %f", msg->position.y);
    ROS_INFO("  position.z: %f", msg->position.z);
    ROS_INFO("  acceleration_or_force.x: %f", msg->acceleration_or_force.x);
    ROS_INFO("  acceleration_or_force.y: %f", msg->acceleration_or_force.y);
    ROS_INFO("  acceleration_or_force.z: %f", msg->acceleration_or_force.z);
    ROS_INFO("  velocity.x: %f", msg->velocity.x);
    ROS_INFO("  velocity.y: %f", msg->velocity.y);
    ROS_INFO("  velocity.z: %f", msg->velocity.z);
    ROS_INFO("  yaw: %f", msg->yaw);
    ROS_INFO("  yaw_rate: %f", msg->yaw_rate);

    ROS_INFO("...redirecting to mavros...");
    tp = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_thread");
    ros::NodeHandle nh;

    ROS_INFO("Offboard PositionTarget Handling Thread");
    ROS_INFO("_______________________________________");
    ROS_INFO("______________---   ---________________");
    ROS_INFO("_______________________________________");

    ros::Subscriber target_pos_sub = nh.subscribe<mavros_msgs::PositionTarget>
            (ros::this_node::getNamespace() + "/mavthread/setpoint_raw/local", 100, target_cb);
    ros::Publisher target_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
            (ros::this_node::getNamespace() + "/mavros/setpoint_raw/local", 100);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    //give tp a starting value
    tp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    tp.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY |
                  mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_AFX |
                  mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                  mavros_msgs::PositionTarget::IGNORE_YAW | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    tp.position.x = 0.0f;
    tp.position.y = 0.0f;
    tp.position.z = 0.0f;
    tp.acceleration_or_force.x = 0.0f;
    tp.acceleration_or_force.y = 0.0f;
    tp.acceleration_or_force.z = 0.0f;
    tp.velocity.x = 0.0f;
    tp.velocity.y = 0.0f;
    tp.velocity.z = 0.0f;
    tp.yaw_rate = 0.0f;

    while(ros::ok()){

        //post target to the vehicle
        target_pos_pub.publish(tp);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
