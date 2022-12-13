#include "move_base_controller.h";

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

BT::NodeStatus MoveBaseController::SendWaypoint::tick() {
    move_base_msgs::MoveBaseGoal target;

    BT::Optional<MoveBaseController::Position2D> opt_goal = getInput<MoveBaseController::Position2D>("goal");
    if (!opt_goal)
        throw BT::RuntimeError("Could not retrieve goal: ", opt_goal.error() );
    
    Position2D goal = opt_goal.value();

    target.target_pose.header.frame_id = "map";
    target.target_pose.header.stamp = ros::Time::now();

    target.target_pose.pose.position.x = goal.x;
    target.target_pose.pose.position.y = goal.y;

    ROS_INFO("Sending goal . . .");

    MoveBaseController::ac->sendGoal(target);

    MoveBaseController::ac->waitForResult();

    if(MoveBaseController::ac->getState() != actionlib::SimpleClientGoalState::REJECTED)
        ROS_INFO("Hooray, the base moved 1 meter forward");
    else
        ROS_INFO("The base failed to move forward 1 meter for some reason");

    return BT::NodeStatus::SUCCESS;
}


