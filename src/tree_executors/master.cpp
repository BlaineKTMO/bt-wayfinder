/**
 * Author: Blaine Oania
 * Filename: master.cpp
 * Date: 11/30/22
 * Package: first_bts
 * Description:
 *  First attempt at creating a behavior tree. This utilizes the normal method
 *  of defining nodes. The nodes are stored in a namespace, and each node is a
 *  class that inherets a node type. I abandoned this method to try the legacy
 *  method, but this way has a few advantages.
 * 
 *  Need to revisit, and try to rewrite turtlebotNavTree using this method.
*/
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <ros/ros.h>

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#include "customNodes.h"

using namespace CustomNodes;

int main(int argc, char **argv)
{   
    BT::BehaviorTreeFactory factory;

    // Register nodes by class
    factory.registerNodeType<DriveForward>("DriveForward");
    factory.registerNodeType<TurnQuarter>("TurnQuarter");

    // Initialize ros
    ros::init(argc, argv, "test_publisher");
    ros::NodeHandle n("~");
    velPub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    // scanSub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000);

    // Create tree
    auto tree = factory.createTreeFromFile("/home/blaine/catkin_ws/src/first_bts/src/trees/tree.xml");

    // Tie to groot
    BT::PublisherZMQ publisher_zmq(tree);

    // Execute tree
    tree.tickRootWhileRunning();

    return 0;
}



