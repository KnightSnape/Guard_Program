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
    auto chassis_yaw = blackboard_ptr_->pos_manager.get_yaw();
    ROS_DEBUG("Start Gimbal Node");
    //ROS_INFO("target_id is %d",(int)blackboard_ptr_->auto_aim_msg.target_id);
    //ROS_WARN("last is %d",(int)blackboard_ptr_->last_gimbal_mode);
    if(blackboard_ptr_->dr16_receive_msg.sw_right != blackboard_ptr_->dr16_receive_msg.SW_UP)
    {
        blackboard_ptr_->last_gimbal_mode = Gimbal_Mode::NO_FORCE;
        return BehaviorState::SUCCESS;
    }
    if(blackboard_ptr_->is_pitch_now_received)
    {
        gimbal_exe_ptr_->get_pitch_now(blackboard_ptr_->pitch_now_msg);
    }
    if(blackboard_ptr_->is_yaw_now_received)
    {
        gimbal_exe_ptr_->get_yaw_now(blackboard_ptr_->yaw_now_msg);
    }
    if(blackboard_ptr_->auto_aim_msg.target_id != 0)
    {
        blackboard_ptr_->gimbal_mode = Gimbal_Mode::FOLLOW_AUTOAIM;
        blackboard_ptr_->autoaim_state_control.remain_control_time = blackboard_ptr_->autoaim_state_control.max_control_time;
        blackboard_ptr_->autoaim_state_control.is_cmd = true;
        ROS_INFO("Get Gimbal AutoAim State");
    }
    else if(blackboard_ptr_->last_gimbal_mode == Gimbal_Mode::FOLLOW_AUTOAIM && (blackboard_ptr_->auto_aim_msg.target_id == 0))
    {
        blackboard_ptr_->autoaim_state_control.remain_control_time = blackboard_ptr_->autoaim_state_control.remain_control_time - blackboard_ptr_->average_time;
        //gimbal_exe_ptr_->operate_state(Gimbal_Mode::STEADY, chassis_yaw);
        blackboard_ptr_->last_gimbal_mode = Gimbal_Mode::FOLLOW_AUTOAIM;
        if(blackboard_ptr_->autoaim_state_control.remain_control_time <= 0)
        {
            blackboard_ptr_->autoaim_state_control.remain_control_time = 0;
            blackboard_ptr_->autoaim_state_control.is_cmd = false;
            blackboard_ptr_->gimbal_mode = Gimbal_Mode::SCANNING;
            blackboard_ptr_->last_gimbal_mode = Gimbal_Mode::STEADY;
        }
        return BehaviorState::SUCCESS;
    }
    else if(blackboard_ptr_->gimbal_mode == Gimbal_Mode::STEADY)
    {
        ROS_DEBUG("Start STEADY");
        //gimbal_exe_ptr_->operate_state(Gimbal_Mode::STEADY, chassis_yaw);
    }
    else if(blackboard_ptr_->gimbal_mode == Gimbal_Mode::LOW_SPEED)
    {   
        ROS_DEBUG("Start LOW_SPEED");
        //gimbal_exe_ptr_->operate_state(Gimbal_Mode::LOW_SPEED, chassis_yaw);
    }
    else if(blackboard_ptr_->gimbal_mode == Gimbal_Mode::HIGH_SPEED)
    {
        ROS_DEBUG("Start HIGH_SPEED");
        //gimbal_exe_ptr_->operate_state(Gimbal_Mode::HIGH_SPEED, chassis_yaw);
    }
    else if(blackboard_ptr_->gimbal_mode == Gimbal_Mode::SCANNING)
    {
        ROS_DEBUG("Start Scanning");
        //gimbal_exe_ptr_->operate_state(Gimbal_Mode::SCANNING, chassis_yaw);
    }
    else if(blackboard_ptr_->gimbal_mode == Gimbal_Mode::FOLLOW_AUTOAIM)
    {
        //std::cout<<blackboard_ptr_->auto_aim_msg.pitch<<std::endl;
        gimbal_exe_ptr_->get_autoaim_target(blackboard_ptr_->auto_aim_msg);
        //gimbal_exe_ptr_->operate_state(Gimbal_Mode::FOLLOW_AUTOAIM, chassis_yaw);
    }
    blackboard_ptr_->last_gimbal_mode = blackboard_ptr_->gimbal_mode;
    return BehaviorState::SUCCESS;

}


}

