/**
 * Author: Blaine Oania
 * Filename: MBTree.cpp
 * Date: 12/13/22
 * Package: first_bts
 * Description:
 *  This tree utilizes the move_base client to navigate. Goals are pulled from
 *  a goals.txt file using GoalController nodes, which are then sent to the 
 *  move_base client as waypoints.
*/
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include <ros/ros.h>
#include <ros/package.h>

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#include "goal_controller.h"
#include "move_base_controller.h"

int main(int argc, char **argv)
{   
    BT::BehaviorTreeFactory factory;

    // Register nodes by class
    factory.registerNodeType<MoveBaseController::SendWaypoint>("SendWaypoint");
    factory.registerNodeType<GoalController::ReadGoals>("ReadGoals");
    factory.registerNodeType<GoalController::RequestGoal>("RequestGoal");
    factory.registerNodeType<GoalController::IncrementIndex>("IncrementIndex");


    // Initialize ros
    ros::init(argc, argv, "test_publisher");
    ros::NodeHandle n("~");

    MoveBaseController::MoveBaseClient mbc("move_base", true);
    while(!mbc.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for move base server to come up.");
    }

    MoveBaseController::initialize(mbc);

    // Create tree
    std::string path = ros::package::getPath("first_bts");
    std::string file = "/src/trees/MBTree.xml";
    path.append(file);
    std::cout << path << std::endl;
    auto tree = factory.createTreeFromFile(path);

    // Tie to groot
    BT::PublisherZMQ publisher_zmq(tree);

    // Execute tree
    tree.tickRootWhileRunning();

    return 0;
}



