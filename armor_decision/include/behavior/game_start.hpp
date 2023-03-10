#pragma once 
#include"decision_tree.h"
#include"chassis_executor.hpp"
#include"log_executor.hpp"
#include"warn_executor.hpp"

namespace decision_tree
{
    class GameStart : public ActionNode
    {
        public:
            GameStart(std::string name,
                        int level,
                        const Blackboard::Ptr &blackboard,
                        const Chassis_executor::Ptr &chassis_executor,
                        const Log_executor::Ptr &log_executor,
                        const Warn_executor::Ptr &warn_executor):
                        ActionNode::ActionNode(name,level,blackboard,chassis_executor,log_executor,warn_executor){}
            BehaviorState Update()
            {
                //使用Selector节点，如果想进入下一个节点，使用FAILURE进入下一个，而使用SUCCESS会跳出
                if(blackboard_ptr_->game_status_msg.game_progress != (uint8_t)4 && blackboard_ptr_->game_status_msg.stage_remain_time > 0)
                {
                    if(blackboard_ptr_->game_status_msg.game_type == (uint8_t)4)
                        blackboard_ptr_->add_blood_state = blackboard_ptr_->Add_Blood_State::EXCELLENT;
                    return BehaviorState::FAILURE;
                }
                //联盟赛游戏结束，状态重启
                if(blackboard_ptr_->is_game_result_received && blackboard_ptr_->game_status_msg.game_type == (uint8_t)4)
                {
                    blackboard_ptr_->add_blood_state = blackboard_ptr_->Add_Blood_State::INITIAL;
                }
                return BehaviorState::SUCCESS;
            }
        private:

    };
}