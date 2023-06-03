#pragma once
#include<ros/ros.h>
#include<tf/tf.h>
#include<geometry_msgs/PoseStamped.h>
#include<mutex>
#include<cmath>
#include<iostream>
#include<opencv2/opencv.hpp>
#include<Eigen/Eigen>
#include"Inference/freeInference.hpp"
#include"utility/node_manager.hpp"
#include"utility/point_graph.hpp"
#include"utility/pos_manager.hpp"
using namespace std;
using namespace cv;

namespace decision_tree
{
    class Blackboard
    {
        public:
            typedef std::shared_ptr<Blackboard> Ptr;

            void reset_flag();
            void init_params();
            Blackboard();

            gary_msgs::DualLoopPIDWithFilter pitch_now_msg;
            gary_msgs::DualLoopPIDWithFilter yaw_now_msg;
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
            nav_msgs::Odometry odometry_msg;

            std::mutex pitch_now_mutex;
            std::mutex yaw_now_mutex;
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
            std::mutex odometry_mutex;

            bool is_pitch_now_received;
            bool is_yaw_now_received;
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
            bool is_odometry_received;

            //手控命令控制
            CMD_Control command_control;
            //发射控制
            Shoot_Control shoot_control;
            //原地控制
            StandBy_Control standBy_control;
            //自瞄控制
            AutoAim_Control autoaim_control;
            //云台自瞄状态控制
            AutoAim_State_Control autoaim_state_control;
            //云台手指令状态
            CMD_Command command_mode;
            //底盘状态
            Chassis_Mode chassis_mode;
            //云台状态
            Gimbal_Mode gimbal_mode;
            //自瞄状态
            AutoAim_Mode autoaim_mode;
            //对应的有向图
            Graph graph;
            //自由决策模式
            FreeInference inference;
            //位置管理器
            Pos_Manager pos_manager;
            //Node管理器
            ROSNodeManager ros_node_manager;
            //当前所在ID
            int now_id;
            //导航所需要的目标
            int navigation_target_id;
            //下一个直线导航目标
            int next_target_id;
            //平均一次处理的时间
            double average_time;

            Eigen::Vector2i sentry_map_point;

        private:

            ros::NodeHandle nh_;

            ros::Subscriber pitch_now_sub;
            ros::Subscriber yaw_now_sub;
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
            ros::Subscriber odometry_sub;

            ros::ServiceServer vision_mode_switch_pub;

            string pitch_now_topic;
            string yaw_now_topic;
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
            string odometry_topic;

            void Pitch_Now_Callback(const gary_msgs::DualLoopPIDWithFilter::ConstPtr msg);
            void Yaw_Now_Callback(const gary_msgs::DualLoopPIDWithFilter::ConstPtr msg);
            void Auto_Aim_Callback(const gary_msgs::AutoAIM::ConstPtr msg);
            void Robot_Hp_Callback(const gary_msgs::RobotHP::ConstPtr msg);
            void Bullet_Remain_Callback(const gary_msgs::BulletRemaining::ConstPtr msg);
            void Game_State_Callback(const gary_msgs::GameStatus::ConstPtr msg);
            void Game_Result_Callback(const gary_msgs::GameResult::ConstPtr msg);
            void Robot_Buff_Callback(const gary_msgs::RobotBuff::ConstPtr msg);
            void Robot_Position_Callback(const gary_msgs::RobotPosition::ConstPtr msg);
            void Robot_Status_Callback(const gary_msgs::RobotStatus::ConstPtr msg);
            void Client_Command_Callback(const gary_msgs::ClientCommand::ConstPtr msg);
            void Client_Receive_Callback(const gary_msgs::ClientReceive::ConstPtr msg);
            void Odometry_Callback(const nav_msgs::Odometry::ConstPtr msg);
            void Client_Command_State_Transform();
    };
}