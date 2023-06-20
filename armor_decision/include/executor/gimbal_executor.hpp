#pragma once
#include<chrono>
#include<iostream>
#include<ros/package.h>
#include"log_executor.hpp"
#include<fstream>
#include<sstream>
#include"utility/utility.hpp"


class Gimbal_executor
{
    public:
        typedef std::shared_ptr<Gimbal_executor> Ptr;

        Gimbal_executor();
        void operate_state(Gimbal_Mode state, double chassis_yaw);
        void get_pitch_now(gary_msgs::DualLoopPIDWithFilter msg);
        void get_yaw_now(gary_msgs::DualLoopPIDWithFilter msg);
        void get_autoaim_target(gary_msgs::AutoAIM msg);
        void get_init_gimbal_yaw_pos();
        void get_target_angle(Eigen::Vector2d target_pos_world,Eigen::Vector2d self_pos_world);

        double gimbal_pitch_min{};
        double gimbal_pitch_max{};
        double gimbal_pitch_now{};
        double gimbal_yaw_now{};
        volatile double gimbal_pitch_set{};
        volatile double gimbal_yaw_set{};
        double yaw_rotate_speed{};
        double pitch_rotate_speed{};
        double pitch_autoaim{};
        double yaw_autoaim{};
        double k_autoaim{};
        double x_offset{};
        double y_offset{};
        double rotate_freq;
        double pitch_upper_threshold;
        double pitch_lower_threshold;
        double yaw_upper_threshold;
        double yaw_lower_threshold;
        bool pitch_sign;
        bool yaw_sign;
        bool is_initial;
        double target_angle{};
        
        Eigen::Vector3d target_world_position;


    private:
        struct gimbal_yaw_set_calc_s
        {
            double cycle;
            double angle;
        }gimbal_yaw_set_calc;
        
        std_msgs::Float64 pitch_msg;
        std_msgs::Float64 yaw_msg;
        ros::Publisher pitch_set_pub;
        ros::Publisher yaw_set_pub;
        Log_executor::Ptr log_exe_ptr_;
        ros::NodeHandle nh;
        volatile double range_diff;
        double init_yaw_pos;
        double yaw_saved;
        double chassis_yaw_saved;
        double pitch_saved;
        Gimbal_Mode last_gimbal_state;

};