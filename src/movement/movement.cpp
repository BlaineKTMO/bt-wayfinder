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

#include "movement.h"

/**
 * 
*/
bool Movement::drive() {
    geometry_msgs::Twist msg;
    int rate = 50;
    ros::Rate loop_rate(rate);

    msg.linear.x = 0.2;
    
    drivePub.publish(msg);

    // // Reset /cmd_vel
    // msg.linear.x = 0;
    // drivePub.publish(msg);

    return true;
}

/**
 * 
*/
bool Movement::turn(double yaw) {
    geometry_msgs::Twist twist_msg;
    nav_msgs::Odometry odom_msg;

    boost::shared_ptr<nav_msgs::Odometry const> odom_ptr;

    tf2::Quaternion start;
    tf2::Quaternion end;
    tf2::Quaternion current;
    tf2::Quaternion rotation;
    // yaw = 1.57;
    rotation.setRPY(0, 0, yaw);

    int rate = 50;
    ros::Rate loop_rate(rate);

    // // Reset /cmd_vel
    // twist_msg.linear.x = 0;
    // twist_msg.linear.y = 0;
    // twist_msg.linear.z = 0;

    // twist_msg.angular.y = 0;
    // twist_msg.angular.z = 0;
    // twist_msg.angular.x = 0;

    // drivePub.publish(twist_msg);

    odom_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", n);
    if (odom_ptr == NULL)
    {
        ROS_INFO("No odom data found.");
    }
    else
        odom_msg = *odom_ptr;

    tf2::fromMsg(odom_msg.pose.pose.orientation, start);
    end = rotation * start;
    end.normalize();

    ROS_INFO("Turning");

    do
    {
        odom_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", n);
        if (odom_ptr == NULL)
            ROS_INFO("No odom data found.");
        else
            odom_msg = *odom_ptr;

        twist_msg = odom_msg.twist.twist;

        tf2::fromMsg(odom_msg.pose.pose.orientation, current);

        // std::cout << "Shortest--------------" << std::endl
        // << tf2::angleShortestPath(end, current) << std::endl 
        // << "---------------" << std::endl;
        if (yaw > 0.7)
            yaw = 0.7;
        else if (yaw <= -0.7)
            yaw = -0.7;

        twist_msg.angular.z = yaw;

        drivePub.publish(twist_msg);

    } while (abs(tf2::angleShortestPath(current, end)) > 0.1 );

    return true;
}

bool Movement::checkCollision() {
    sensor_msgs::LaserScan scan;
    boost::shared_ptr<sensor_msgs::LaserScan const> scan_ptr;
    float avgDistance = 0.;
    
    scan_ptr = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", n);

    if (scan_ptr == NULL)
        ROS_INFO("No laser scan found.");
    else
        scan = *scan_ptr;

    // for (auto it = scan.ranges.begin(); it != scan.ranges.begin() + 25; it++) 
    // {
    //     avgDistance += *it;
    // }

    // for (auto it = scan.ranges.end(); it != scan.ranges.end() - 25; it-- )
    // {
    //     avgDistance += *it;
    // }

    // avgDistance /= 50;

    ROS_INFO("Checking for collision");

    int count = 0;

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

