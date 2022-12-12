#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#include <vector>
    
#include "customNodes.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h" 

// Register nodes with the BT
BT_REGISTER_NODES(factory)
{
    CustomNodes::RegisterNodes(factory);
}


namespace CustomNodes
{
    // Define publisher nodes
    ros::Publisher velPub;
    ros::Subscriber scanSub;

// bool DriveForward::init(ros::NodeHandle &n) {
//     DriveForward::n = n;
    
//     return true;
// }

// Override of method in SyncActionNode 
BT::NodeStatus DriveForward::tick() 
{
    geometry_msgs::Twist msg;
    boost::shared_ptr<sensor_msgs::LaserScan const> shared_ptr;
    sensor_msgs::LaserScan laserScan;
    int rate = 50;
    ros::Rate loop_rate(rate);
    ROS_INFO("I RAN");
    
    msg.linear.x = 0.5;
    msg.linear.y = 0;
    msg.linear.z = 0;

    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;

    for(int i = 0; i < 50;  i++) {
        // float avgDistance = 0;

        // velPub.publish(msg);
        // shared_ptr = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", n);
        // if(shared_ptr == NULL)
        //     ROS_INFO("No point cloud messages recieved.");
        // else 
        //     laserScan = *shared_ptr;

        // for(std::vector<float>::iterator it = laserScan.ranges.begin(); it != laserScan.ranges.begin() + 25; it++)
        // {
        //     avgDistance += *it;
        // }
        // for(std::vector<float>::iterator it = laserScan.ranges.end(); it != laserScan.ranges.end() - 25; it--)
        // {
        //     avgDistance += *it;
        // }
        // avgDistance /= 50;
        
        loop_rate.sleep();
        ros::spinOnce();
    }

    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;

    velPub.publish(msg);
    
    return BT::NodeStatus::SUCCESS;
}

DriveForward::~DriveForward()
{}

// bool TurnQuarter::init(ros::NodeHandle &n) {
//     TurnQuarter::n = n;

//     return true;
// }


// Override method of SyncActionNode
BT::NodeStatus TurnQuarter::tick()
{
    // geometry_msgs::Twist msg;

    // msg.linear.x = -0.5;
    // msg.linear.y = 0;
    // msg.linear.z = 0;

    // msg.angular.x = 0;
    // msg.angular.y = 0;
    // msg.angular.z = 0;

    // int rate = 50;
    // ros::Rate loop_rate(rate);  

    // // msg.angular.x = 10;

    // for(int i = 0; i < 50; i++) {
    //     scanSub.publish(msg);
    //     loop_rate.sleep();
    // }
    
    return BT::NodeStatus::SUCCESS;
}

TurnQuarter::~TurnQuarter()
{}
}