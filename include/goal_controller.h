/**
 * Author: Blaine Oania
 * Filename: goal_controller.h
 * Date: 12/13/22
 * Package: first_bts
 * Description:
 *  Handles retrieving, passing, and selection of goals. This implementation
 *  is completely divorced from ROS, aside from handling errors and reading package
 *  directory. 
*/
#ifndef GOAL_CONTROLLER_H
#define GOAL_CONTROLLER_H

#include <ros/ros.h>
#include <ros/package.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <fstream>
#include <vector>

namespace GoalController {

    class ReadGoals : public BT::SyncActionNode {
        public:
        ReadGoals(const std::string& name, const BT::NodeConfiguration& config) 
            : BT::SyncActionNode(name, config) {}

        static BT::PortsList providedPorts() {
            return { 
                BT::InputPort<std::string>("file"), 
                BT::OutputPort<std::string>("goals"),
                BT::OutputPort<double>("num_goals") 
            };
        }

        BT::NodeStatus tick() override;

    };

    class RequestGoal : public BT::SyncActionNode {
        public:
        RequestGoal(const std::string& name, const BT::NodeConfiguration& config) 
            : BT::SyncActionNode(name, config) {}

        static BT::PortsList providedPorts() {
            return { 
                BT::InputPort<std::string>("goals"),
                BT::InputPort<double>("num_goals"),
                BT::InputPort<double>("index"),
                BT::OutputPort<std::string>("goal") 
            };
        }

        BT::NodeStatus tick() override;
    };

    class IncrementIndex : public BT::SyncActionNode {
        public:
            IncrementIndex(const std::string& name, const BT::NodeConfiguration& config) 
                : BT::SyncActionNode(name, config) {}

            static BT::PortsList providedPorts() {
                return { 
                    BT::InputPort<double>("index"),
                    BT::OutputPort<double>("index_out"),
                };
            }

            BT::NodeStatus tick() override;
    };
}

#endif
