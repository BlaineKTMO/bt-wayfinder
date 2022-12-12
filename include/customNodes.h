#ifndef CUSTOM_NODES_H
#define CUSTOM_NODES_H

#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace CustomNodes {

    extern ros::Publisher velPub;
    extern ros::Subscriber scanSub;

    class DriveForward : public BT::SyncActionNode
    {
        // private:
        //     static ros::NodeHandle &n;

        public:
            DriveForward(const std::string& name) :
                BT::SyncActionNode(name, {})
            {}
            virtual ~DriveForward();
            BT::NodeStatus tick() override;
            static bool init(ros::NodeHandle &node);

        
    };

    
    class TurnQuarter : public BT::SyncActionNode
    {
        // private:
        //     static ros::NodeHandle &n;

        public:
            TurnQuarter(const std::string& name) :
                BT::SyncActionNode(name, {})
            {}
            virtual ~TurnQuarter();
            BT::NodeStatus tick() override;
            static bool init(ros::NodeHandle &node);
    };

    inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
    {
        factory.registerNodeType<DriveForward>("DriveForward");
        factory.registerNodeType<TurnQuarter>("TurnQuarter");
    }
}

#endif