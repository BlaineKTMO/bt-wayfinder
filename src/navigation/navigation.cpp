/**
 * Author: Blaine Oania
 * Filename: navigation.cpp
 * Date: 11/30/22
 * Package: first_bts
 * Description:
 *  Navigation client that handles planning
*/
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include <string>

#include "navigation.h"

double Navigation::findYaw(geometry_msgs::Point goal) {
    nav_msgs::Odometry odom_msg;

    boost::shared_ptr<nav_msgs::Odometry const> odom_ptr;
    geometry_msgs::Point current_pos;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped stampedTransform;

    geometry_msgs::PoseStamped stampedPose;
    geometry_msgs::PoseStamped outputStampedPose;

    geometry_msgs::PoseStamped stampedGoal;
    geometry_msgs::PoseStamped outputStampedGoal;

    odom_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", n);
    if (odom_ptr == NULL)
        ROS_INFO("No odom data found.");
    else
        odom_msg = *odom_ptr;

    // tf2::fromMsg(odom_msg.pose.pose.position, current_pos);
    current_pos.x = odom_msg.pose.pose.position.x;
    current_pos.y = odom_msg.pose.pose.position.y;

    /**
     * Need to use stampedPoses and transform to base_footprint for proper yaw calculations
     * 
     * dont yet know why
    */

    stampedPose.header.frame_id = "odom";
    stampedPose.header.stamp = ros::Time(0);
    stampedPose.pose.position = current_pos;

    stampedGoal.header.frame_id = "odom";
    stampedGoal.header.stamp = ros::Time(0);
    stampedGoal.pose.position = goal;

    tfBuffer.transform<geometry_msgs::PoseStamped>(stampedPose, outputStampedPose, "base_footprint", ros::Duration(3.0));
    tfBuffer.transform<geometry_msgs::PoseStamped>(stampedGoal, outputStampedGoal, "base_footprint", ros::Duration(3.0));
    
    double dx = outputStampedGoal.pose.position.x - outputStampedPose.pose.position.x;
    double dy = outputStampedGoal.pose.position.y - outputStampedPose.pose.position.y;

    double yaw = atan2(dy, dx);
    std::cout << "Yaw: " << yaw << std::endl;

    return yaw;
}

bool Navigation::navDrive(geometry_msgs::Point goal) {
    nav_msgs::Odometry odom_msg;

    boost::shared_ptr<nav_msgs::Odometry const> odom_ptr;

    geometry_msgs::Point current_pos;
    geometry_msgs::Twist vel;
    
    geometry_msgs::PoseStamped stampedPose;
    geometry_msgs::PoseStamped outputStampedPose;

    geometry_msgs::PoseStamped stampedGoal;
    geometry_msgs::PoseStamped outputStampedGoal;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped stampedTransform;

    odom_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", n);
    if (odom_ptr == NULL)
        ROS_INFO("No odom data found.");
    else
        odom_msg = *odom_ptr;

    // tf2::fromMsg(odom_msg.pose.pose.position, current_pos);
    current_pos.x = odom_msg.pose.pose.position.x;
    current_pos.y = odom_msg.pose.pose.position.y;

    /**
     * Don't need to transform to base_footprint to get proper
    */

    // stampedPose.header.frame_id = "odom";
    // stampedPose.header.stamp = ros::Time::now();
    // stampedPose.pose.position = current_pos;

    // stampedGoal.header.frame_id = "odom";
    // stampedGoal.header.stamp = ros::Time::now();
    // stampedGoal.pose.position = goal;

    // tfBuffer.transform<geometry_msgs::PoseStamped>(stampedPose, outputStampedPose, "base_footprint", ros::Duration(3.0));
    // tfBuffer.transform<geometry_msgs::PoseStamped>(stampedGoal, outputStampedGoal, "base_footprint", ros::Duration(3.0));
    
    // double dx = outputStampedGoal.pose.position.x - outputStampedPose.pose.position.x;
    // double dy = outputStampedGoal.pose.position.y - outputStampedPose.pose.position.y;

    double dx = goal.x - current_pos.x;
    double dy = goal.y - current_pos.x;

    double speed = pow( (dx * dx) + (dy * dy), 0.5);

    if (speed > 1)
        speed = 1;
    else if (speed < 0.1)
        speed = 0;

    vel.linear.x = speed;

    drivePub.publish(vel);

    return true;
}