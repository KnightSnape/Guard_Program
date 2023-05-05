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
        
        void initParams(ros::NodeHandle &nh);
        void operate_state(int state);
        void get_pitch_now(gary_msgs::DualLoopPIDWithFilter msg);
        void get_yaw_now(gary_msgs::DualLoopPIDWithFilter msg);

        float gimbal_pitch_min;
        float gimbal_pitch_max;

        float gimbal_pitch_now;
        float gimbal_yaw_now;

        float yaw_rotate_speed;
        float pitch_rotate_speed;
        float rotate_freq;

        float pitch_upper_threshold;
        float pitch_lower_threshold;

        float yaw_upper_threshold;
        float yaw_lower_threshold;

        bool pitch_sign;
        bool yaw_sign;

        ros::Publisher pitch_set_pub;
        ros::Publisher yaw_set_pub;
        Log_executor::Ptr log_exe_ptr_;

};