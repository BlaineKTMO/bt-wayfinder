/**
 * Author: Blaine Oania
 * Filename: topic_controller.h
 * Date: 12/13/22
 * Package: first_bts
 * Description:
 *  Handles publishing and subscribing to ros topics.
*/
#ifndef TOPIC_CONTROLLER_H
#define TOPIC_CONTROLLER_H

#include <ros/ros.h>
#include <ros/package.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace TopicController {

    struct Position2D 
    {
        double x;
        double y;
    };

    class PublishPoseEstimate : public BT::SyncActionNode {
        public:
            PublishPoseEstimate(const std::string& name, const BT::NodeConfiguration& config, ros::Publisher initialpose_pub)
                : BT::SyncActionNode(name, config), 
                initialpose_pub(initialpose_pub) {}

            static BT::PortsList providedPorts() {
                return {
                    BT::InputPort<Position2D>("initial_pose")
                };
            }

            BT::NodeStatus tick() override;

        private:
            ros::Publisher initialpose_pub;

    };
}

#endif