#ifndef MOVE_BASE_CONTROLLER_H
#define MOVE_BASE_CONTROLLER_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace MoveBaseController {

    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    
    static MoveBaseClient* ac;

    struct Position2D {
        double x;
        double y;
    };

    class SendWaypoint : public BT::SyncActionNode
    {
        public:
            SendWaypoint(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)  {}
           
            static BT::PortsList providedPorts() {
                return { BT::InputPort<std::string>("goal") };
            }

            BT::NodeStatus tick() override;
        

    };
};



#endif