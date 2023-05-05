#include"autoaim_switch.hpp"

namespace decision_tree
{

AutoAim_Switch::AutoAim_Switch(std::string name,
                               int level,
                               const Blackboard::Ptr &blackboard,
                               const Chassis_executor::Ptr &chassis_executor,
                               const Gimbal_executor::Ptr &gimbal_executor,
                               const Log_executor::Ptr &log_executor,
                               const AutoAim_executor::Ptr &autoaim_executor):
                               ActionNode::ActionNode(name,level,blackboard,chassis_executor,gimbal_executor,log_executor,autoaim_executor){}

BehaviorState AutoAim_Switch::Update()
{
    if(blackboard_ptr_->is_client_command_received)
    {
        //优先攻击
        if(blackboard_ptr_->command_mode == CMD_Command::PRIORITY_ATTACK)
        {
            //云台手控制模式
            blackboard_ptr_->autoaim_mode = AutoAim_Mode::LISTEN_CAPTAIN;
            blackboard_ptr_->autoaim_control.is_cmd = true;
            blackboard_ptr_->autoaim_control.remain_control_time = blackboard_ptr_->autoaim_control.max_control_time;
            int target_robot_id = (int)blackboard_ptr_->client_command_msg.target_robot_id;
            if(target_robot_id > 100)
                target_robot_id -= 100;
            blackboard_ptr_->autoaim_control.target_id = target_robot_id;
        }
    }
    if(blackboard_ptr_->autoaim_mode == AutoAim_Mode::LISTEN_CAPTAIN)
    {
        blackboard_ptr_->autoaim_control.remain_control_time = blackboard_ptr_->autoaim_control.remain_control_time - blackboard_ptr_->average_time;
        autoaim_exe_ptr_->pub_autoaim_state(4,blackboard_ptr_->autoaim_control.target_id);
        ROS_DEBUG("pub captain target autoaim mode");
        if(blackboard_ptr_->autoaim_control.remain_control_time <= 0)
        {
            blackboard_ptr_->autoaim_control.remain_control_time = 0;
            blackboard_ptr_->autoaim_control.is_cmd = false;
            blackboard_ptr_->autoaim_mode = AutoAim_Mode::NORMAL;
            return BehaviorState::SUCCESS;
        }
    }

    if(blackboard_ptr_->autoaim_mode == AutoAim_Mode::OUTPOST)
    {
        autoaim_exe_ptr_->pub_autoaim_state(1,6);
        ROS_DEBUG("pub outpost autoaim mode");
    }

    if(blackboard_ptr_->autoaim_mode == AutoAim_Mode::KILL_TARGET)
    {
        autoaim_exe_ptr_->pub_autoaim_state(2,0);
        ROS_DEBUG("pub kill_target autoaim mode");
    }

    if(blackboard_ptr_->autoaim_mode == AutoAim_Mode::NORMAL)
    {
        autoaim_exe_ptr_->pub_autoaim_state(3,0);
        ROS_DEBUG("pub normal autoaim mode");
    }

    return BehaviorState::SUCCESS;

}

}