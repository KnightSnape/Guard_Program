#include"game_start.hpp"

namespace decision_tree
{

GameStart::GameStart(std::string name,
                    int level,
                    const Blackboard::Ptr &blackboard,
                    const Chassis_executor::Ptr &chassis_executor,
                    const Gimbal_executor::Ptr &gimbal_executor,
                    const Log_executor::Ptr &log_executor,
                    const AutoAim_executor::Ptr &autoaim_executor):
                    ActionNode::ActionNode(name,level,blackboard,chassis_executor,gimbal_executor,log_executor,autoaim_executor)
                    {
                        is_start = false;
                    }

BehaviorState GameStart::Update()
{
    if(blackboard_ptr_->game_status_msg.game_progress!=(uint8_t)4)
    {
        ROS_DEBUG("not start yet");
        blackboard_ptr_->chassis_mode = Chassis_Mode::INITIAL;
        blackboard_ptr_->gimbal_mode = Gimbal_Mode::INITIAL;
        blackboard_ptr_->autoaim_mode = AutoAim_Mode::INITIAL;
        return BehaviorState::FAILURE;
    }
    else
    {
        if(!is_start)
        {
            ROS_DEBUG("Start Race");
            //赛场开始时初始化底盘，云台，自瞄状态
            blackboard_ptr_->chassis_mode = Chassis_Mode::STANDBY;
            blackboard_ptr_->gimbal_mode = Gimbal_Mode::SCANNING;
            blackboard_ptr_->autoaim_mode = AutoAim_Mode::NORMAL;
            is_start = true;
        }
        return BehaviorState::SUCCESS;
    }
}

}