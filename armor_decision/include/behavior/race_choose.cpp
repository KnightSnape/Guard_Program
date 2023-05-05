#include"race_choose.hpp"

namespace decision_tree
{
    
    
Race_Choose::Race_Choose(std::string name,
                         int level,
                         const Blackboard::Ptr &blackboard,
                         const Chassis_executor::Ptr &chassis_executor,
                         const Gimbal_executor::Ptr &gimbal_executor,
                         const Log_executor::Ptr &log_executor,
                         const AutoAim_executor::Ptr &autoaim_executor):
                         SelectorNode::SelectorNode(name,level,blackboard,chassis_executor,gimbal_executor,log_executor,autoaim_executor){}


BehaviorState Race_Choose::Update()
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

}