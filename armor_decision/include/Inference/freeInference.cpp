#include"freeInference.hpp"

namespace decision_tree
{

FreeInference::FreeInference()
{

}

void FreeInference::init_param(std::string path)
{
    std::string final_path = path + "config.yaml";
    YAML::Node Config = YAML::LoadFile(final_path);
    mode = Config["mode"].as<int>();
}

void FreeInference::get_autoaim(const gary_msgs::AutoAIM &msg)
{
    auto_aim_msg = msg;
}

void FreeInference::get_bulletremain(const gary_msgs::BulletRemaining &msg)
{
    bullet_remaining_msg = msg;
}

void FreeInference::get_gamestatus(const gary_msgs::GameStatus &msg)
{
    game_status_msg = msg;
}

void FreeInference::get_robotbuff(const gary_msgs::RobotBuff &msg)
{
    robot_buff_msg = msg;
}

void FreeInference::get_robothp(const gary_msgs::RobotHP &msg)
{
    robot_hp_msg = msg;
}

void FreeInference::get_robotposition(const gary_msgs::RobotPosition &msg)
{
    robot_position_msg = msg;
}

void FreeInference::main_process(int &target,Gimbal_Mode& gimbal_mode,AutoAim_Mode& autoaim_mode)
{
    
}

}

