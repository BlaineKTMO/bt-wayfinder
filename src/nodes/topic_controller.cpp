/**
 * Author: Blaine Oania
 * Filename: topic_controller.cpp
 * Date: 12/14/22
 * Package: first_bts
 * Description:
 *  Handles publishing and subscribing to ros topics.
*/

#include "topic_controller.h"

BT::NodeStatus TopicController::PublishPoseEstimate::tick() {
    geometry_msgs::PoseWithCovarianceStamped pose_estimate;

    tf2::Quaternion target_quat;

    // Creating a valid orientation quaternion
    target_quat.setRPY(0, 0, 0);
    target_quat.normalize();

    BT::Optional<TopicController::Position2D> opt_initialpose = getInput<TopicController::Position2D>("initial_pose");
    if (!opt_initialpose)
        throw BT::RuntimeError("Could not retrieve initialpose: ", opt_initialpose.error() );
    
    Position2D goal = opt_initialpose.value();

    // Initializing pose estimate
    pose_estimate.header.frame_id = "map";
    pose_estimate.header.stamp = ros::Time::now();

    pose_estimate.pose.pose.position.x = goal.x;
    pose_estimate.pose.pose.position.y = goal.y;

    pose_estimate.pose.pose.orientation.w = target_quat.getW();
    pose_estimate.pose.pose.orientation.x = target_quat.getX();
    pose_estimate.pose.pose.orientation.y = target_quat.getY();
    pose_estimate.pose.pose.orientation.z = target_quat.getZ();

    std::cout << "target: " << target_quat << std::endl;

    ROS_INFO("Sending pose estimate . . .");

    initialpose_pub.publish(pose_estimate);
    
    return BT::NodeStatus::SUCCESS;
}


