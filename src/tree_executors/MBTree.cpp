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
#include "movement_client.h"
#include "topic_controller.h"

namespace BT
{
    template <> inline TopicController::Position2D convertFromString(StringView str)
    {
        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        if (parts.size() != 2)
        {
            throw RuntimeError("invalid input)");
        }
        else
        {
            TopicController::Position2D output;
            output.x     = convertFromString<double>(parts[0]);
            output.y     = convertFromString<double>(parts[1]);
            return output;
        }
    }
};

int main(int argc, char **argv)
{   
    BT::BehaviorTreeFactory factory;

    // Initialize ros
    ros::init(argc, argv, "test_publisher");
    ros::NodeHandle n("~");

    Movement move_client(n);
    move_client.setDrivePub(n.advertise<geometry_msgs::Twist>("/cmd_vel", 100));

    ros::Publisher initialpose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

    auto turnLambda = [&move_client](BT::TreeNode& parentNode) -> BT::NodeStatus {
        double yaw = 3.14;
        bool flag = move_client.turn(yaw);

        return flag ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    };

    BT::NodeBuilder PublishPoseEstimateBuilder = [initialpose_pub](const std::string& name, const BT::NodeConfiguration& config)
    {
        return std::make_unique<TopicController::PublishPoseEstimate>(name, config, initialpose_pub);
    };

    MoveBaseController::MoveBaseClient mbc("move_base", true);
    while(!mbc.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for move base server to come up.");
    }

    MoveBaseController::initialize(mbc);

    // Register nodes by class
    factory.registerNodeType<MoveBaseController::SendWaypoint>("SendWaypoint");
    factory.registerNodeType<GoalController::ReadGoals>("ReadGoals");
    factory.registerNodeType<GoalController::RequestGoal>("RequestGoal");
    factory.registerNodeType<GoalController::IncrementIndex>("IncrementIndex");
    factory.registerBuilder<TopicController::PublishPoseEstimate>("PublishPoseEstimate", PublishPoseEstimateBuilder);
    
    factory.registerSimpleAction("Turn", turnLambda);


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



