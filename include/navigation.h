#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "ros/ros.h"
#include "geometry_msgs/Point.h"

struct Position2D {
    double x;
    double y;
};

class Navigation {
    private:
        ros::NodeHandle &n;
        ros::Publisher drivePub;
     
    public:
        Navigation(ros::NodeHandle &n, ros::Publisher drivePub) : n(n), drivePub(drivePub)
         {} 
    
        double findYaw(geometry_msgs::Point goal);
        bool navDrive(geometry_msgs::Point goal);
        
};

#endif