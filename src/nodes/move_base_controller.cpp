/**
 * Author: Blaine Oania
 * Filename: move_base_controller.cpp
 * Date: 12/13/22
 * Package: first_bts
 * Description:
 *  Nodes for handling move_base requests.
*/

#include "move_base_controller.h"
#include "tf2/LinearMath/Quaternion.h"

namespace BT
{
    template <> inline MoveBaseController::Position2D convertFromString(StringView str)
    {
        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        if (parts.size() != 2)
        {
            throw RuntimeError("invalid input)");
        }
        else
        {
            MoveBaseController::Position2D output;
            output.x     = convertFromString<double>(parts[0]);
            output.y     = convertFromString<double>(parts[1]);
            return output;
        }
    }
};

bool MoveBaseController::initialize(MoveBaseClient &mbc) {
    ac = &mbc;
    return true;
}

BT::NodeStatus MoveBaseController::SendWaypoint::tick() {
    move_base_msgs::MoveBaseGoal target;
    tf2::Quaternion target_quat;

    // Creating a valid orientation quaternion
    target_quat.setZ(0.5);
    target_quat.setW(0.8);
    target_quat.normalize();

    BT::Optional<MoveBaseController::Position2D> opt_goal = getInput<MoveBaseController::Position2D>("goal");
    if (!opt_goal)
        throw BT::RuntimeError("Could not retrieve goal: ", opt_goal.error() );
    
    Position2D goal = opt_goal.value();

    // Initializing goal
    target.target_pose.header.frame_id = "map";
    target.target_pose.header.stamp = ros::Time::now();

    target.target_pose.pose.position.x = goal.x;
    target.target_pose.pose.position.y = goal.y;
    
    target.target_pose.pose.orientation.w = target_quat.getW();
    target.target_pose.pose.orientation.w = target_quat.getX();
    target.target_pose.pose.orientation.w = target_quat.getY();
    target.target_pose.pose.orientation.w = target_quat.getZ();

    // std::cout << "target: " << target << std::endl;

    ROS_INFO("Sending goal . . .");

    ac->sendGoal(target);
    ac->waitForResult();

    if(ac->getState() != actionlib::SimpleClientGoalState::REJECTED)
        ROS_INFO("Goal has been reached.");
    else
        ROS_INFO("Unable to reach goal");

    return BT::NodeStatus::SUCCESS;
}


