/**
 * Author: Blaine Oania
 * Filename: navigation.cpp
 * Date: 12/15/22
 * Package: first_bts
 * Description:
 *  Navigation client that handles planning. These methods are wrapped by lamdas
 *  then turned into nodes.
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
    
    // Odom variables
    nav_msgs::Odometry odom_msg;
    boost::shared_ptr<nav_msgs::Odometry const> odom_ptr;

    // Position variables
    geometry_msgs::Point current_pos;
    geometry_msgs::PoseStamped stampedPose;
    geometry_msgs::PoseStamped stampedGoal;

    // Transform handlers
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Transformed position variables
    geometry_msgs::PoseStamped outputStampedPose;
    geometry_msgs::PoseStamped outputStampedGoal;

    // Get odometry information
    odom_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", n);
    if (odom_ptr == NULL)
        ROS_INFO("No odom data found.");
    else
        odom_msg = *odom_ptr;

    // Store position information from current odom
    // tf2::fromMsg(odom_msg.pose.pose.position, current_pos);
    current_pos.x = odom_msg.pose.pose.position.x;
    current_pos.y = odom_msg.pose.pose.position.y;

    /**
     * Need to use stampedPoses and transform to base_footprint for proper yaw calculations
     * 
     * dont yet know why
    */

    // Define stamped poses for position transformation
    stampedPose.header.frame_id = "odom";
    stampedPose.header.stamp = ros::Time(0);
    stampedPose.pose.position = current_pos;

    stampedGoal.header.frame_id = "odom";
    stampedGoal.header.stamp = ros::Time(0);
    stampedGoal.pose.position = goal;

    // Get transformations odom -> base_footprint
    tfBuffer.transform<geometry_msgs::PoseStamped>(stampedPose, outputStampedPose, "base_footprint", ros::Duration(3.0));
    tfBuffer.transform<geometry_msgs::PoseStamped>(stampedGoal, outputStampedGoal, "base_footprint", ros::Duration(3.0));

    // Find deltas
    double dx = outputStampedGoal.pose.position.x - outputStampedPose.pose.position.x;
    double dy = outputStampedGoal.pose.position.y - outputStampedPose.pose.position.y;

    // Calculate yaw
    double yaw = atan2(dy, dx);

    std::cout << "stampedPose: " << stampedPose << std::endl;
    std::cout << "outputStampedPose: " << outputStampedPose << std::endl;

    std::cout << "stampedGoal: " << stampedGoal << std::endl;
    std::cout << "outputStampedGoal: " << outputStampedGoal << std::endl;

    return yaw;
}

bool Navigation::navDrive(geometry_msgs::Point goal) {
    // Odom variables
    nav_msgs::Odometry odom_msg;
    boost::shared_ptr<nav_msgs::Odometry const> odom_ptr;

    // Vel variables
    geometry_msgs::Twist vel;

    // Position variables
    geometry_msgs::Point current_pos;
    geometry_msgs::PoseStamped stampedPose;
    geometry_msgs::PoseStamped stampedGoal;

    // Transform handlers
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Transformed position variables
    geometry_msgs::PoseStamped outputStampedPose;
    geometry_msgs::PoseStamped outputStampedGoal;

    // Get odom information
    odom_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", n);
    if (odom_ptr == NULL)
        ROS_INFO("No odom data found.");
    else
        odom_msg = *odom_ptr;

    // Store position info from current odom
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

    // Find deltas
    double dx = goal.x - current_pos.x;
    double dy = goal.y - current_pos.x;

    // Calculate distance
    double speed = pow( (dx * dx) + (dy * dy), 0.5);

    // Find speed as a funtion of velocity (Clamp [0-1])
    if (speed > 1)
        speed = 1;
    else if (speed < 0.1)
        speed = 0;

    // Publish cmd_vel
    vel.linear.x = speed;
    drivePub.publish(vel);

    return true;
}