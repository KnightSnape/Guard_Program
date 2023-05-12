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
        void operate_state(int state);
        void get_pitch_now(gary_msgs::DualLoopPIDWithFilter msg);
        void get_yaw_now(gary_msgs::DualLoopPIDWithFilter msg);

        double gimbal_pitch_min{};
        double gimbal_pitch_max{};
        double gimbal_pitch_now{};
        double gimbal_yaw_now{};
        double yaw_rotate_speed{};
        double pitch_rotate_speed{};
        double rotate_freq;
        double pitch_upper_threshold;
        double pitch_lower_threshold;
        double yaw_upper_threshold;
        double yaw_lower_threshold;
        bool pitch_sign;
        bool yaw_sign;

    private:
        std_msgs::Float64 pitch_msg;
        std_msgs::Float64 yaw_msg;
        ros::Publisher pitch_set_pub;
        ros::Publisher yaw_set_pub;
        Log_executor::Ptr log_exe_ptr_;
        ros::NodeHandle nh;

};