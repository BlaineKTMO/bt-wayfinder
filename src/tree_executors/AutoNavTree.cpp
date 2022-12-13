/**
 * Author: Blaine Oania
 * Filename: master.cpp
 * Date: 12/13/22
 * Package: first_bts
 * Description:
 *  TO BE WRITTEN
*/
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <ros/ros.h>

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#include "move_base_controller.h"

int main(int argc, char **argv)
{   
    BT::BehaviorTreeFactory factory;

    // Register nodes by class
    factory.registerNodeType<MoveBaseController::SendWaypoint>("SendWaypoint");

    // Initialize ros
    ros::init(argc, argv, "auto_nav_tree");
    ros::NodeHandle n("~");
    
    MoveBaseController::MoveBaseClient mbc("move_base", true);
    MoveBaseController::ac = &mbc;

    // Create tree
    auto tree = factory.createTreeFromFile("/home/blaine/catkin_ws/src/first_bts/src/trees/AutoNavTree.xml");

    // Tie to groot
    BT::PublisherZMQ publisher_zmq(tree);

    // Execute tree
    tree.tickRootWhileRunning();

    return 0;
}