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

    factory.registerNodeType<DriveForward>("DriveForward");
    factory.registerNodeType<TurnQuarter>("TurnQuarter");

    ros::init(argc, argv, "test_publisher");
    ros::NodeHandle n("~");


    velPub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    // scanSub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000);

    auto tree = factory.createTreeFromFile("/home/blaine/catkin_ws/src/first_bts/src/tree.xml");

    BT::PublisherZMQ publisher_zmq(tree);

    tree.tickRootWhileRunning();

    return 0;
}



