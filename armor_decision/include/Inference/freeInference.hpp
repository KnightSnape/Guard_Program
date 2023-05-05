#pragma once
#include"utility/utility.hpp"
#include"decisionInference.hpp"

namespace decision_tree
{

class FreeInference
{
    public:
        FreeInference();
        void init_param(std::string path);
        void get_autoaim(const gary_msgs::AutoAIM &msg);
        void get_bulletremain(const gary_msgs::BulletRemaining &msg);
        void get_gamestatus(const gary_msgs::GameStatus &msg);
        void get_robotbuff(const gary_msgs::RobotBuff &msg);
        void get_robothp(const gary_msgs::RobotHP &msg);
        void get_robotposition(const gary_msgs::RobotPosition &msg);
        void main_process(int &target,Gimbal_Mode& gimbal_mode,AutoAim_Mode& autoaim_mode);
    private:

        int mode;
        InferenceDecision net_decision;
        gary_msgs::AutoAIM auto_aim_msg;
        gary_msgs::BulletRemaining bullet_remaining_msg;
        gary_msgs::GameStatus game_status_msg;
        gary_msgs::RobotBuff robot_buff_msg;
        gary_msgs::RobotHP robot_hp_msg;
        gary_msgs::RobotPosition robot_position_msg;

};

}

