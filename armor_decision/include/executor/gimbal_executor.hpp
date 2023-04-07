#pragma once
#include<chrono>
#include<iostream>
#include<ros/package.h>
#include<fstream>
#include<sstream>

class Gimbal_executor
{
    public:
        typedef std::shared_ptr<Gimbal_executor> Ptr;

        void Initial_process()
        {

        }

        void low_speed_process()
        {

        }

        void scanning_process()
        {

        }

        void warn_captain_process()
        {

        }

        void follow_autoaim_process()
        {
            
        }

        ros::NodeHandle nh_;
        Log_executor::Ptr log_exe_ptr_;
};