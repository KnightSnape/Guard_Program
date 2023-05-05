#include"gimbal_switch.hpp"

namespace decision_tree
{

Gimbal_Switch::Gimbal_Switch(std::string name,
                               int level,
                               const Blackboard::Ptr &blackboard,
                               const Chassis_executor::Ptr &chassis_executor,
                               const Gimbal_executor::Ptr &gimbal_executor,
                               const Log_executor::Ptr &log_executor,
                               const AutoAim_executor::Ptr &autoaim_executor):
                               ActionNode::ActionNode(name,level,blackboard,chassis_executor,gimbal_executor,log_executor,autoaim_executor){}

BehaviorState Gimbal_Switch::Update()
{
    if(blackboard_ptr_->is_auto_aim_received)
    {
        blackboard_ptr_->gimbal_mode = Gimbal_Mode::FOLLOW_AUTOAIM;
    }
    if(blackboard_ptr_->is_client_command_received)
    {
        blackboard_ptr_->gimbal_mode = Gimbal_Mode::WARN_CAPTAIN;
    }
    if(blackboard_ptr_->gimbal_mode == Gimbal_Mode::WARN_CAPTAIN)
    {
        gimbal_exe_ptr_->operate_state(1);
    }
    if(blackboard_ptr_->gimbal_mode == Gimbal_Mode::LOW_SPEED)
    {   
        gimbal_exe_ptr_->operate_state(2);
    }
    if(blackboard_ptr_->gimbal_mode == Gimbal_Mode::HIGH_SPEED)
    {
        gimbal_exe_ptr_->operate_state(3);
    }
    if(blackboard_ptr_->gimbal_mode == Gimbal_Mode::SCANNING)
    {
        gimbal_exe_ptr_->operate_state(4);
    }
    if(blackboard_ptr_->gimbal_mode == Gimbal_Mode::FOLLOW_AUTOAIM)
    {
    
        if(!blackboard_ptr_->is_auto_aim_received)
        {

            return BehaviorState::SUCCESS;
        }
    }

}


}

