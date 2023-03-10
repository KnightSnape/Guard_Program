#pragma once 
#include"decision_tree.h"
#include"chassis_executor.hpp"
#include"log_executor.hpp"
#include"warn_executor.hpp"

namespace decision_tree
{
    class Add_Blood : public ActionNode
    {
        public:
            Add_Blood(std::string name,
                        int level,
                        const Blackboard::Ptr &blackboard,
                        const Chassis_executor::Ptr &chassis_executor,
                        const Log_executor::Ptr &log_executor,
                        const Warn_executor::Ptr &warn_executor):
                        ActionNode::ActionNode(name,level,blackboard,chassis_executor,log_executor,warn_executor){}

            BehaviorState Update()
            {
                //只有前四分钟才有效果，如果在最后一分钟，直接跳过
                if(blackboard_ptr_->game_status_msg.stage_remain_time <= (uint16_t)60)
                {
                    ROS_DEBUG("Last One minute");
                    return BehaviorState::FAILURE;
                }
                //对于联盟赛的状态下，补血操作如下
                if(blackboard_ptr_->game_status_msg.game_type == (uint8_t)4)
                {
                    //以下是判断是否切换状态
                    if(blackboard_ptr_->add_blood_state == blackboard_ptr_->Add_Blood_State::EXCELLENT)
                    {
                        //判断是否有必要回血(下面的式子很可能要改)
                        int remain_time = (int)(blackboard_ptr_->game_status_msg.stage_remain_time) - 60;
                        int remain_blood = (int)blackboard_ptr_->robot_status_msg.remain_hp;
                        float add_blood_point = remain_blood - log(240 - remain_time) * (240 - remain_time);
                        //如果血量判断过小，切换状态
                        if(add_blood_point < add_blood_threshold)
                        {
                            //切换成FIND_D_TAG行为
                            blackboard_ptr_->add_blood_state = blackboard_ptr_->Add_Blood_State::FIND_D_TAG;
                        }
                        //如果不符合跳转
                        else
                            return BehaviorState::FAILURE;
                    }
                    else if(blackboard_ptr_->add_blood_state == blackboard_ptr_->Add_Blood_State::FIND_D_TAG)
                    {
                        //成功锁定D_tag
                        if(blackboard_ptr_->auto_aim_msg.pitch <= lock_out_pitch_threshold && 
                             blackboard_ptr_->auto_aim_msg.yaw <= lock_out_yaw_threshold  &&
                             !(((int)(blackboard_ptr_->auto_aim_msg.target_id))&1))
                        {
                            blackboard_ptr_->add_blood_state = blackboard_ptr_->Add_Blood_State::GOTO_D_TAG;
                        }
                    }
                    else if(blackboard_ptr_->add_blood_state == blackboard_ptr_->Add_Blood_State::GOTO_D_TAG)
                    {
                        //检测到回血
                        if(blackboard_ptr_->is_robot_buff_received&&
                           blackboard_ptr_->robot_buff_msg.robot_replenishing_blood)
                        {
                            blackboard_ptr_->add_blood_state = blackboard_ptr_->Add_Blood_State::RECOVERING;
                        }
                    }

                    else if(blackboard_ptr_->add_blood_state == blackboard_ptr_->Add_Blood_State::RECOVERING)
                    {
                        //已经补满就可以切识别A的tag
                        if(blackboard_ptr_->is_robot_status_received && 
                        blackboard_ptr_->robot_status_msg.remain_hp == blackboard_ptr_->robot_status_msg.max_hp)
                        {
                            blackboard_ptr_->add_blood_state = blackboard_ptr_->Add_Blood_State::FIND_A_TAG;
                        }
                    }

                    else if(blackboard_ptr_->add_blood_state == blackboard_ptr_->Add_Blood_State::FIND_A_TAG)
                    {
                        //成功锁定A_tag
                        if(blackboard_ptr_->auto_aim_msg.pitch <= lock_out_pitch_threshold && 
                             blackboard_ptr_->auto_aim_msg.yaw <= lock_out_yaw_threshold  &&
                             ((int)(blackboard_ptr_->auto_aim_msg.target_id))&1)
                        {
                            blackboard_ptr_->add_blood_state = blackboard_ptr_->Add_Blood_State::GOTO_A_TAG;
                        }
                    }

                    else if(blackboard_ptr_->add_blood_state == blackboard_ptr_->Add_Blood_State::GOTO_A_TAG)
                    {
                        //成功返回既定区域
                        if(blackboard_ptr_->auto_aim_msg.target_distance <= min_diatance_threshold)
                        {
                            blackboard_ptr_->add_blood_state = blackboard_ptr_->Add_Blood_State::EXCELLENT;
                        }
                    }
                    return Do_Update_State();
                }
                else
                {
                    //尚未开发
                    return BehaviorState::FAILURE;
                }
            }

            BehaviorState Do_Update_State()
            {
                
                if(blackboard_ptr_->add_blood_state == blackboard_ptr_->Add_Blood_State::EXCELLENT)
                {
                    blackboard_ptr_->chassis_mode = blackboard_ptr_->Chassis_Mode::INITIAL;
                    blackboard_ptr_->gimbal_mode = blackboard_ptr_->Gimbal_Mode::FIND_ARMY;
                }
                else if(blackboard_ptr_->add_blood_state == blackboard_ptr_->Add_Blood_State::FIND_D_TAG)
                {
                    blackboard_ptr_->chassis_mode = blackboard_ptr_->Chassis_Mode::STANDBY_ROTATE;
                    blackboard_ptr_->gimbal_mode = blackboard_ptr_->Gimbal_Mode::FIND_TAG;
                }
                else if(blackboard_ptr_->add_blood_state == blackboard_ptr_->Add_Blood_State::GOTO_D_TAG)
                {
                    blackboard_ptr_->chassis_mode = blackboard_ptr_->Chassis_Mode::FOLLOW_GIMBAL;
                    blackboard_ptr_->gimbal_mode = blackboard_ptr_->Gimbal_Mode::FOLLOW_TAG;
                }
                  
                else if(blackboard_ptr_->add_blood_state == blackboard_ptr_->Add_Blood_State::RECOVERING)
                {
                    blackboard_ptr_->chassis_mode = blackboard_ptr_->Chassis_Mode::STANDBY_ROTATE;
                    blackboard_ptr_->gimbal_mode = blackboard_ptr_->Gimbal_Mode::FIND_ARMY;
                }
                else if(blackboard_ptr_->add_blood_state == blackboard_ptr_->Add_Blood_State::FIND_A_TAG)
                {
                    blackboard_ptr_->chassis_mode = blackboard_ptr_->Chassis_Mode::INITIAL;
                    blackboard_ptr_->gimbal_mode = blackboard_ptr_->Gimbal_Mode::FIND_TAG;
                }
                else if(blackboard_ptr_->add_blood_state == blackboard_ptr_->Add_Blood_State::GOTO_A_TAG)
                {
                    blackboard_ptr_->chassis_mode = blackboard_ptr_->Chassis_Mode::FOLLOW_GIMBAL;
                    blackboard_ptr_->gimbal_mode = blackboard_ptr_->Gimbal_Mode::FOLLOW_TAG;
                }
                else
                {
                    return BehaviorState::FAILURE;
                }
            }


        private:

            const float add_blood_threshold = 500;

            const float lock_out_pitch_threshold = 0.02;

            const float lock_out_yaw_threshold = 0.02;

            const float min_diatance_threshold = 1.0;
    };
}