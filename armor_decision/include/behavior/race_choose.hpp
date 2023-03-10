#pragma once 
#include"decision_tree.h"
#include"chassis_executor.hpp"
#include"log_executor.hpp"
#include"warn_executor.hpp"

namespace decision_tree
{
    class Race_Choose : public SelectorNode
    {
        public:
            Race_Choose(std::string name,
                        int level,
                        const Blackboard::Ptr &blackboard,
                        const Chassis_executor::Ptr &chassis_executor,
                        const Log_executor::Ptr &log_executor,
                        const Warn_executor::Ptr &warn_executor):
                        SelectorNode::SelectorNode(name,level,blackboard,chassis_executor,log_executor,warn_executor){}

            BehaviorState Update()
            {
                //RMUL 
                if(blackboard_ptr_->game_status_msg.game_type == (uint8_t)4)
                {
                    if(child_node_ptr_list_[0]->Run() == BehaviorState::SUCCESS)
                        return BehaviorState::SUCCESS;
                    return BehaviorState::FAILURE;
                }
                //RMUC
                else if(blackboard_ptr_->game_status_msg.game_type == (uint8_t)1)
                {
                    if(child_node_ptr_list_[1]->Run() == BehaviorState::SUCCESS)
                        return BehaviorState::SUCCESS;
                    return BehaviorState::FAILURE;
                }
                else
                {
                    ROS_WARN("wrong race");
                    return BehaviorState::FAILURE;
                }
            }
        private:
    };
}