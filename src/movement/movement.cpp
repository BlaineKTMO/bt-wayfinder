/**
 * Author: Blaine Oania
 * Filename: movement.cpp
 * Date: 11/18/22
 * Package: first_bts
 * Description:
 *  Movement client that handles ros
*/
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <string>

#include "movement_client.h"

/**
 * Start driving
*/
bool Movement::drive() {
    geometry_msgs::Twist msg;

    msg.linear.x = 0.2;
    
    drivePub.publish(msg);

    // // Reset /cmd_vel
    // msg.linear.x = 0;
    // drivePub.publish(msg);

    return true;
}

bool Movement::turn(double yaw) {
    // Vel variables
    geometry_msgs::Twist twist_msg;

    // Odom variables
    nav_msgs::Odometry odom_msg;
    boost::shared_ptr<nav_msgs::Odometry const> odom_ptr;

    // Rotation variables
    tf2::Quaternion start;
    tf2::Quaternion end;
    tf2::Quaternion current;
    tf2::Quaternion rotation;
    
    // Set the rotation quaternion
    rotation.setRPY(0, 0, yaw);

    // // Reset /cmd_vel
    // twist_msg.linear.x = 0;
    // twist_msg.linear.y = 0;
    // twist_msg.linear.z = 0;

    // twist_msg.angular.y = 0;
    // twist_msg.angular.z = 0;
    // twist_msg.angular.x = 0;

    // drivePub.publish(twist_msg);

    // Get odom information
    odom_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", n);
    if (odom_ptr == NULL)
    {
        ROS_INFO("No odom data found.");
    }
    else
        odom_msg = *odom_ptr;

    // Store current orientation into start
    tf2::fromMsg(odom_msg.pose.pose.orientation, start);

    // Calculate ending orientation from rotation quaternion and starting orientation
    end = rotation * start;
    end.normalize();

    // Keep looping while the current and ending orientations are more than 0.1 rad apart
    do
    {
        // Get odom information
        odom_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", n);
        if (odom_ptr == NULL)
            ROS_INFO("No odom data found.");
        else
            odom_msg = *odom_ptr;

        // Store current vel
        twist_msg = odom_msg.twist.twist;

        // Store current orientation quaternion
        tf2::fromMsg(odom_msg.pose.pose.orientation, current);

        // Clamp rotation speed [-0.7, 0.7]
        if (yaw > 0.7)
            yaw = 0.7;
        else if (yaw <= -0.7)
            yaw = -0.7;
        twist_msg.angular.z = yaw;

        // Publish to /cmd_vel
        drivePub.publish(twist_msg);

    } while (abs(tf2::angleShortestPath(current, end)) > 0.1 );

    return true;
}

bool Movement::checkCollision() {
    // Scan variables
    sensor_msgs::LaserScan scan;
    boost::shared_ptr<sensor_msgs::LaserScan const> scan_ptr;

    float avgDistance = 0.;
    
    // Get scan information
    scan_ptr = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", n);
    if (scan_ptr == NULL)
        ROS_INFO("No laser scan found.");
    else
        scan = *scan_ptr;

    /**
     * Deprecated collision detection system
     */

    // for (auto it = scan.ranges.begin(); it != scan.ranges.begin() + 25; it++) 
    // {
    //     avgDistance += *it;
    // }

    // for (auto it = scan.ranges.end(); it != scan.ranges.end() - 25; it-- )
    // {
    //     avgDistance += *it;
    // }

    // avgDistance /= 50;

    // Store number of kept scans
    int count = 0;

    /**
     * Collision Detection
     * 
     * If distance > 2, discard that information. If avg is greater than 0.5, 
     * there is no collision.
    */
    for(int i = 0; i < 20; i++)
    {  
        // Guard
        if(scan.ranges[i] > 2) {}

        avgDistance += scan.ranges[i];
        count +=1;
        
    }

    for(int i = 359; i > 339; i--)
    {
        if(scan.ranges[i] > 2) {}

        avgDistance += scan.ranges[i];
        count += 1;

    }

    avgDistance /= count;
    // std::string log = std::to_string(avgDistance);

    if(avgDistance < 0.5)
        return false;

    ROS_INFO("No Collision");
    return true;
}



void Movement::setDrivePub(ros::Publisher drivePub) {
    this->drivePub = drivePub;
}

