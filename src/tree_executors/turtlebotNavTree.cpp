/**
 * Author: Blaine Oania
 * Filename: turtlebotNavTree.cpp
 * Date: 12/15/22
 * Package: first_bts
 * Description:
 *  Third attempt at creating a behavior tree. This tree will take a set of 2D
 *  coordinates as a goal and navigate to it. There is no collision avoidance
 *  programmed. Ports are used to transfer information about heading and the goals.
 * 
 *  This is using the legacy method for wrapping prewritten code in lambdas, then
 *  dynamically creating tree nodes from the lambda. Will be looking to rewrite this
 *  with the "proper method"
*/
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/tree_node.h>
#include <behaviortree_cpp_v3/behavior_tree.h>

#include <ros/ros.h>

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

#include "turtlebotNodes.h"
#include "movement.h"
#include "navigation.h"

// Method override for parsing port string into data
namespace BT
{
    template <>
    Position2D convertFromString(StringView key)
    {
        auto parts = BT::splitString(key, ';');
        if (parts.size() != 2)
        {
            throw RuntimeError("invalid input)");
        }
        else
        {
            Position2D output;
            output.x = convertFromString<double>(parts[0]);
            output.y = convertFromString<double>(parts[1]);
            return output;
        }
    }
};

int main(int argc, char **argv)
{   
    // Ros initialization stuff
    ros::init(argc, argv, "behavior_tree");
    ros::NodeHandle n("~");

    ros::Publisher drivePub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

    Movement move_client(n);
    Navigation nav_client(n, drivePub);

    move_client.setDrivePub(drivePub);

    /**
     * Lambda definitions
    */

    auto driveLambda = [&nav_client](BT::TreeNode& parent_node) -> BT::NodeStatus {
        Position2D goal; 
        geometry_msgs::Point point_goal;

        // Get port information
        parent_node.getInput("goal", goal);
        
        // Convert to ROS data type
        point_goal.x = goal.x;
        point_goal.y = goal.y;
        
        // Call navigation method
        bool flag = nav_client.navDrive(point_goal);

        // Set return status
        return flag ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    };
    
    auto turnLambda = [&move_client](BT::TreeNode& parent_node) -> BT::NodeStatus {
        double yaw;
        parent_node.getInput("yaw", yaw);
        bool flag = move_client.turn(yaw);

        return flag ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    };

    // auto checkCollisionLambda = [&move_client](BT::TreeNode& parentNode) -> BT::NodeStatus {
    //     bool flag = move_client.checkCollision();

    //     return flag ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
    // };

    // auto checkOpenLambda = [&move_client](BT::TreeNode& parentNode) -> BT::NodeStatus {
    //     bool flag = move_client.checkCollision();

    //     return flag ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    // };

    auto findYawLamda = [&nav_client](BT::TreeNode& parent_node) -> BT::NodeStatus {
        Position2D goal; 
        geometry_msgs::Point point_goal;
        parent_node.getInput("goal", goal);
        point_goal.x = goal.x;
        point_goal.y = goal.y;
        double yaw = nav_client.findYaw(point_goal);
        parent_node.setOutput("yaw", yaw);

        bool flag=true;

        return flag ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    };

        BT::BehaviorTreeFactory factory;

    /**
     * Defining behavior tree ports
    */

    BT::PortsList goalPort = {BT::InputPort<Position2D>("goal")};
    BT::PortsList yawPort = {BT::InputPort<double>("yaw")};
    BT::PortsList outputPorts = {BT::InputPort<Position2D>("goal"), BT::OutputPort<double>("yaw") };

    /**
     * Defining nodes from lambdas
    */

    // factory.registerSimpleCondition("CheckCollision", checkCollisionLambda);
    // factory.registerSimpleCondition("CheckOpen", checkOpenLambda);
    factory.registerSimpleAction("navDrive", driveLambda, goalPort);
    factory.registerSimpleAction("Turn", turnLambda, yawPort);
    factory.registerSimpleAction("findYaw", findYawLamda, outputPorts);

    // Create tree
    auto tree = factory.createTreeFromFile("/home/blaine/catkin_ws/src/first_bts/src/trees/turtlebotNavTree.xml");

    // Tie to groot
    BT::PublisherZMQ publisher_zmq(tree);

    // Execute tree
    tree.tickRootWhileRunning();

    return 0;
}
