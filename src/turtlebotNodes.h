#ifndef TURTLE_BOT_NODES_H
#define TURTLE_BOT_NODES_H

#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace TurtlebotNodes {

    inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
    {
        // factory.registerNodeType<DriveForward>("DriveForward");
        // factory.registerNodeType<TurnQuarter>("TurnQuarter");
    }

    class Drive : public BT::SyncActionNode
    {
        private:
            static ros::NodeHandle &n;

        public:
            Drive(const std::string& name) :
                BT::SyncActionNode(name, {})
                {}

            virtual ~Drive();
            BT::NodeStatus tick() override;
    };
}

#endif