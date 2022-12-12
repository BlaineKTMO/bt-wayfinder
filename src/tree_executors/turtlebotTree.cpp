/**
 * Author: Blaine Oania
 * Filename: turtlebotTree.cpp
 * Date: 12/5/22
 * Package: first_bts
 * Description:
 *  Second attempt at creating behavior tree. Rudimentary tree that can drive,
 *  then continously checks for objects in front. If one is detected, the robot
 *  will turn a set degree amount then check for collision again. This will repeat
 *  4 times until failure.
 * 
 *  This is using the legacy method for wrapping prewritten code in lambdas, then
 *  dynamically creating tree nodes from the lambda. 
*/
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/tree_node.h>

#include <ros/ros.h>

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#include "turtlebotNodes.h"
#include "movement.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "behavior_tree");
    ros::NodeHandle n("~");

    Movement move_client(n);

    move_client.setDrivePub(n.advertise<geometry_msgs::Twist>("/cmd_vel", 100));

    auto driveLambda = [&move_client](BT::TreeNode& parentNode) -> BT::NodeStatus {
        bool flag = move_client.drive();

        return flag ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    };
    
    auto turnLambda = [&move_client](BT::TreeNode& parentNode) -> BT::NodeStatus {
        double yaw = 1.5;
        bool flag = move_client.turn(yaw);

        return flag ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    };

    auto checkCollisionLambda = [&move_client](BT::TreeNode& parentNode) -> BT::NodeStatus {
        bool flag = move_client.checkCollision();

        return flag ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
    };

    auto checkOpenLambda = [&move_client](BT::TreeNode& parentNode) -> BT::NodeStatus {
        bool flag = move_client.checkCollision();

        return flag ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    };

    BT::BehaviorTreeFactory factory;

    factory.registerSimpleCondition("CheckCollision", checkCollisionLambda);
    factory.registerSimpleCondition("CheckOpen", checkOpenLambda);

    factory.registerSimpleAction("Drive", driveLambda);
    factory.registerSimpleAction("Turn", turnLambda);

    auto tree = factory.createTreeFromFile("/home/blaine/catkin_ws/src/first_bts/src/trees/turtlebot_tree.xml");

    BT::PublisherZMQ publisher_zmq(tree);

    tree.tickRootWhileRunning();

    return 0;
}
