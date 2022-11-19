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
#include "sensor_msgs/LaserScan.h"

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
bool Movement::turn() {
    geometry_msgs::Twist msg;
    int rate = 50;
    ros::Rate loop_rate(rate);

    // Reset /cmd_vel
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;

    msg.angular.y = 0;
    msg.angular.z = 0;
    msg.angular.x = 0;

    drivePub.publish(msg);

    msg.angular.z = 0.5;

    ROS_INFO("Driving");

    for(int i = 0; i < 100; i++)
    {
        drivePub.publish(msg);
        loop_rate.sleep();
    }

    // Reset /cmd_vel
    msg.angular.z = 0;
    drivePub.publish(msg);

    return true;
}

bool Movement::checkCollision() {
    sensor_msgs::LaserScan scan;
    boost::shared_ptr<sensor_msgs::LaserScan const> scan_ptr;
    float avgDistance = 0.;
    
    scan_ptr = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", n);

    if (scan_ptr == NULL)
        ROS_INFO("No laser scan found");
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

