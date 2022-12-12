
/**
 * Author: Blaine Oania
 * Date: 11/18
 * Filename: TurtleBotNodes.cpp
 * Package: first_bts
 * Description:
 *  Collection of nodes for turtlebot behavior tree.
*/


#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#include <vector>
    
#include "turtlebotNodes.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h" 

// Register nodes with the BT
BT_REGISTER_NODES(factory)
{
    TurtlebotNodes::RegisterNodes(factory);
}


namespace TurtleBotNodes
{
    BT::NodeStatus tick() {
        ros::Publisher drivePub;

        return BT::NodeStatus::SUCCESS;
    }
}