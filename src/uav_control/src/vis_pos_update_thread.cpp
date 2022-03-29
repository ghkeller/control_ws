/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL

 * This thread is dedicated to posting the offboard positions (as suggested by
 * the Mavros documentation)
 * 
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::PoseStamped ps_current;

void vis_pos_update_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ROS_INFO("Recieved setpoint position message contents: ");
    ROS_INFO("  position.x: %f", msg->pose.position.x);
    ROS_INFO("  position.y: %f", msg->pose.position.y);
    ROS_INFO("  position.z: %f", msg->pose.position.z);
    ROS_INFO("  orientation.x: %f", msg->pose.orientation.x);
    ROS_INFO("  orientation.y: %f", msg->pose.orientation.y);
    ROS_INFO("  orientation.z: %f", msg->pose.orientation.z);
    ROS_INFO("  orientation.w: %f", msg->pose.orientation.w);

    ROS_INFO("...redirecting to mavros...");
    ps_current = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vis_pos_update_thread");
    ros::NodeHandle nh;

    ROS_INFO("Vision Position Update Thread");
    ROS_INFO("_______________________________________");
    ROS_INFO("______________---   ---________________");
    ROS_INFO("_______________________________________");

    ros::Subscriber current_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavthread/vision_pose/pose", 100, vis_pos_update_cb);
    ros::Publisher current_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/vision_pose/pose", 100);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    //give tp a starting value
    ps_current.header.frame_id = "VSLAM";
    ps_current.header.stamp = ros::Time::now();
    ps_current.pose.position.x = 0.0f;
    ps_current.pose.position.y = 0.0f;
    ps_current.pose.position.z = 0.0f;
    ps_current.pose.orientation.x = 0.0f;
    ps_current.pose.orientation.y = 0.0f;
    ps_current.pose.orientation.z = 0.707f;
    ps_current.pose.orientation.w = 0.707f;

    while(ros::ok()){

        //post target to the vehicle
        current_pos_pub.publish(ps_current);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}