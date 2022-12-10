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
    ros::init(argc, argv, "behavior_tree");
    ros::NodeHandle n("~");

    ros::Publisher drivePub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

    Movement move_client(n);
    Navigation nav_client(n, drivePub);

    move_client.setDrivePub(drivePub);

    auto driveLambda = [&nav_client](BT::TreeNode& parent_node) -> BT::NodeStatus {
        Position2D goal; 
        geometry_msgs::Point point_goal;
        parent_node.getInput("goal", goal);
        point_goal.x = goal.x;
        point_goal.y = goal.y;
        bool flag = nav_client.navDrive(point_goal);
                std::cout << point_goal.x;

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
    BT::PortsList goalPort = {BT::InputPort<Position2D>("goal")};
    BT::PortsList ports = {BT::InputPort<double>("yaw")};
    BT::PortsList outputPorts = {BT::InputPort<Position2D>("goal"), BT::OutputPort<double>("yaw"), };
    // factory.registerSimpleCondition("CheckCollision", checkCollisionLambda);
    // factory.registerSimpleCondition("CheckOpen", checkOpenLambda);

    factory.registerSimpleAction("navDrive", driveLambda, goalPort);
    factory.registerSimpleAction("Turn", turnLambda, ports);
    factory.registerSimpleAction("findYaw", findYawLamda, outputPorts);

    auto tree = factory.createTreeFromFile("/home/blaine/catkin_ws/src/first_bts/src/turtlebotNavTree.xml");

    BT::PublisherZMQ publisher_zmq(tree);

    tree.tickRootWhileRunning();

    return 0;
}
