#pragma once
#include<ros/ros.h>
#include<tf/tf.h>
#include<geometry_msgs/PoseStamped.h>
#include<mutex>
#include<cmath>
#include<iostream>
#include<opencv2/opencv.hpp>
#include<Eigen/Eigen>
#include"gary_msgs/AutoAIM.h"
#include"gary_msgs/RobotHP.h"
#include"gary_msgs/BulletRemaining.h"
#include"gary_msgs/GameStatus.h"
#include"gary_msgs/GameResult.h"
#include"gary_msgs/RobotBuff.h"
#include"gary_msgs/RobotPosition.h"
#include"gary_msgs/RobotStatus.h"
#include"gary_msgs/ClientCommand.h"
#include"gary_msgs/ClientReceive.h"
#include"gary_msgs/VisionModeSwitch.h"
#include"gary_msgs/VisionModeSwitchRequest.h"
#include"gary_msgs/VisionModeSwitchResponse.h"
using namespace std;
using namespace cv;

namespace decision_tree
{
    class Blackboard
    {
        public:
            typedef std::shared_ptr<Blackboard> Ptr;

            enum class RemainTime
            {
                ZERO = 0,//未启动
                INIT = 1,//初始位置前往
                MIDDLE = 2,//6:30-4:00阶段使用
                HOT = 3,//4:00之后的哨兵高潮时间
            };

            //联盟赛使用的状态
            enum class Add_Blood_State
            {
                INITIAL = 0,//基本的初始化状态
                EXCELLENT = 1,//状态良好，在比赛开始时变换成该状态
                FIND_D_TAG = 2,//血量不足，寻找D_tag(不快速的转动云台，直到找到D_tag并稳定识别之后才能切换)
                GOTO_D_TAG = 3,//找到D_tag之后，会一直保持这个状态直到检测到自己补血。
                RECOVERING = 4,//这个状态下会一直检测自己的血量状态，直到满血之后才会搜寻tag位置
                FIND_A_TAG = 5,//原理和FIND_D_TAG相同
                GOTO_A_TAG = 6,//原理和GOTO_D_TAG相同(停下来切换是依据视觉解算距离判断防止视野过小)
            };
            //底盘状态
            enum class Chassis_Mode 
            {
                INITIAL = 0,//无变化，静止
                STANDBY_ROTATE = 1,//原地
                FOLLOW_GIMBAL = 2,//跟随云台(前进)
            }; 
            //云台状态
            enum class Gimbal_Mode
            {
                INITIAL = 0,//无变化，静止(但可以测试自瞄，然而不能发弹丸)
                FIND_ARMY = 1,//扫视范围中的敌人，如果有直接使用自瞄，并开启小陀螺
                FIND_TAG = 2,//在该模式下，虽然扫视策略相同，但是自瞄目标将会变成TAG
                ATTACK_ARMY = 3,//锁死敌方的时候使用，此时云台只听从自瞄
                FOLLOW_TAG = 4,//锁TAG时使用
            };

            void reset_flag()
            {
                is_auto_aim_received = false;
                is_robot_hp_received = false;
                is_bullet_remain_received = false;
                is_game_status_received = false;
                is_game_result_received = false;
                is_robot_buff_received = false;
                is_robot_position_received = false;
                is_robot_status_received = false;
                is_client_command_received = false;
                is_client_receive_received = false;
            }

            Blackboard():
            is_auto_aim_received(false),
            is_robot_hp_received(false),
            is_bullet_remain_received(false),
            is_game_status_received(false),
            is_game_result_received(false),
            is_robot_buff_received(false),
            is_robot_position_received(false),
            is_robot_status_received(false),
            is_client_command_received(false),
            is_client_receive_received(false)
            {
                add_blood_state = Add_Blood_State::INITIAL;
                chassis_mode = Chassis_Mode::INITIAL;
                gimbal_mode = Gimbal_Mode::INITIAL;

                auto_aim_sub = nh_.subscribe(auto_aim_sub_topic,10,&Blackboard::Auto_Aim_Callback,this);
                robot_hp_sub = nh_.subscribe(robot_hp_sub_topic,10,&Blackboard::Robot_Hp_Callback,this);
                bullet_remain_sub = nh_.subscribe(bullet_remain_sub_topic,10,&Blackboard::Bullet_Remain_Callback,this);
                game_status_sub = nh_.subscribe(game_state_sub_topic,10,&Blackboard::Game_State_Callback,this);
                game_result_sub = nh_.subscribe(game_result_sub_topic,10,&Blackboard::Game_Result_Callback,this);
                robot_buff_sub = nh_.subscribe(robot_buff_sub_topic,10,&Blackboard::Robot_Buff_Callback,this);
                robot_position_sub = nh_.subscribe(robot_position_sub_topic,10,&Blackboard::Robot_Position_Callback,this);
                robot_status_sub = nh_.subscribe(robot_status_sub_topic,10,&Blackboard::Robot_Status_Callback,this);
                client_command_sub = nh_.subscribe(client_command_sub_topic,10,&Blackboard::Client_Command_Callback,this);
                client_receive_sub = nh_.subscribe(client_receive_sub_topic,10,&Blackboard::Client_Receive_Callback,this);

                //vision_mode_switch_pub = nh_.advertiseService(vision_mode_switch_topic,10);
            }

            gary_msgs::AutoAIM auto_aim_msg;
            gary_msgs::RobotHP robot_hp_msg;
            gary_msgs::BulletRemaining bullet_remain_msg;
            gary_msgs::GameStatus game_status_msg;
            gary_msgs::GameResult game_result_msg;
            gary_msgs::RobotBuff robot_buff_msg;
            gary_msgs::RobotPosition robot_position_msg;
            gary_msgs::RobotStatus robot_status_msg;
            gary_msgs::ClientCommand client_command_msg;
            gary_msgs::ClientReceive client_receive_msg;
            gary_msgs::VisionModeSwitch vision_mode_switch_srv;

            std::mutex auto_aim_mutex;
            std::mutex robot_hp_mutex;
            std::mutex bullet_remain_mutex;
            std::mutex game_status_mutex;
            std::mutex game_result_mutex;
            std::mutex robot_buff_mutex;
            std::mutex robot_position_mutex;
            std::mutex robot_status_mutex;
            std::mutex client_command_mutex;
            std::mutex client_receive_mutex;
            std::mutex vision_mode_switch_mutex;

            bool is_auto_aim_received;
            bool is_robot_hp_received;
            bool is_bullet_remain_received;
            bool is_game_status_received;
            bool is_game_result_received;
            bool is_robot_buff_received;
            bool is_robot_position_received;
            bool is_robot_status_received;
            bool is_client_command_received;
            bool is_client_receive_received;

            //回血过程状态
            Add_Blood_State add_blood_state;   
            //底盘状态
            Chassis_Mode chassis_mode;
            //云台状态
            Gimbal_Mode gimbal_mode;

        private:

            ros::NodeHandle nh_;

            ros::Subscriber auto_aim_sub;
            ros::Subscriber robot_hp_sub;
            ros::Subscriber bullet_remain_sub;
            ros::Subscriber game_status_sub;
            ros::Subscriber game_result_sub;
            ros::Subscriber robot_buff_sub;
            ros::Subscriber robot_position_sub;
            ros::Subscriber robot_status_sub;
            ros::Subscriber client_command_sub;
            ros::Subscriber client_receive_sub;

            ros::ServiceServer vision_mode_switch_pub;

            string auto_aim_sub_topic;
            string robot_hp_sub_topic;
            string bullet_remain_sub_topic;
            string game_state_sub_topic;
            string game_result_sub_topic;
            string robot_buff_sub_topic;
            string robot_position_sub_topic;
            string robot_status_sub_topic;
            string client_command_sub_topic;
            string client_receive_sub_topic;
            string vision_mode_switch_topic;

            void Auto_Aim_Callback(const gary_msgs::AutoAIM::ConstPtr msg)
            {
                auto_aim_mutex.lock();
                auto_aim_msg = *msg;
                is_auto_aim_received = true;
                auto_aim_mutex.unlock();
            }

            void Robot_Hp_Callback(const gary_msgs::RobotHP::ConstPtr msg)
            {
                robot_hp_mutex.lock();
                robot_hp_msg = *msg;
                is_robot_hp_received = true;
                robot_hp_mutex.unlock();
            }

            void Bullet_Remain_Callback(const gary_msgs::BulletRemaining::ConstPtr msg)
            {
                bullet_remain_mutex.lock();
                bullet_remain_msg = *msg;
                is_bullet_remain_received = true;
                bullet_remain_mutex.unlock();
            }

            void Game_State_Callback(const gary_msgs::GameStatus::ConstPtr msg)
            {
                game_status_mutex.lock();
                game_status_msg = *msg;
                is_game_result_received = true;
                game_status_mutex.unlock();
            }

            void Game_Result_Callback(const gary_msgs::GameResult::ConstPtr msg)
            {
                game_result_mutex.lock();
                game_result_msg = *msg;
                is_game_result_received = true;
                game_result_mutex.unlock();
            }

            void Robot_Buff_Callback(const gary_msgs::RobotBuff::ConstPtr msg)
            {
                robot_buff_mutex.lock();
                robot_buff_msg = *msg;
                is_robot_buff_received = true;
                robot_buff_mutex.unlock();
            }

            void Robot_Position_Callback(const gary_msgs::RobotPosition::ConstPtr msg)
            {
                robot_position_mutex.lock();
                robot_position_msg = *msg;
                is_robot_buff_received = true;
                robot_position_mutex.unlock();
            }

            void Robot_Status_Callback(const gary_msgs::RobotStatus::ConstPtr msg)
            {
                robot_status_mutex.lock();
                robot_status_msg = *msg;
                is_robot_status_received = true;
                robot_status_mutex.unlock();
            }
            
            void Client_Command_Callback(const gary_msgs::ClientCommand::ConstPtr msg)
            {
                client_command_mutex.lock();
                client_command_msg = *msg;
                is_client_command_received = true;
                client_command_mutex.unlock();
            }

            void Client_Receive_Callback(const gary_msgs::ClientReceive::ConstPtr msg)
            {
                client_receive_mutex.lock();
                client_receive_msg = *msg;
                is_client_receive_received = true;
                client_receive_mutex.unlock();
            }
    };
}