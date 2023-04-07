#pragma once
#include<ros/ros.h>
#include"log_executor.hpp"
#include"gimbal_executor.hpp"
#include<geometry_msgs/PoseStamped.h>


class Chassis_executor
{
    public:
        typedef std::shared_ptr<Chassis_executor> Ptr;

        void initial_process()
        {
            
        }

        void standby_process()
        {

        }

        void listen_captain_process()
        {

        }

        void follow_gimbal_process()
        {

        }

        void free_captain_process()
        {
            
        }

        ros::NodeHandle nh_;
        Log_executor::Ptr log_exe_ptr_;
        std::stringstream str;
};