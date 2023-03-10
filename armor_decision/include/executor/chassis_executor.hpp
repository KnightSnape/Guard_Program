#pragma once
#include<ros/ros.h>
#include"log_executor.hpp"
#include<geometry_msgs/PoseStamped.h>


class Chassis_executor
{
    public:
        enum class ChassisStatus
        {
            IDLE,
            RUNNING
        };
        typedef std::shared_ptr<Chassis_executor> Ptr;

        void Stop()
        {
            if(status_ == ChassisStatus::RUNNING)
            status_ = ChassisStatus::IDLE;
        }
        void Goto(double pos_x,double pos_y,double pos_z,double pos_theta,bool Immediately)
        {
            
        }
        void GotoBack(double pos_x,double pos_y,double pos_z,double pos_theta,bool Immediately)
        {

        }
        void GotoSwing(double pos_x,double pos_y,double pos_z,double pos_theta,bool Immediately)
        {

        }
        void GotoSwingBack(double pos_x,double pos_y,double pos_z,double pos_theta,bool Immediately)
        {

        }
        void Rotate(double pos_theta)
        {

        }
        void Spin()
        {
            
        }
        ChassisStatus status_;
        private:
        void SendDataToPlan(uint8_t command,double pos_x,double pos_y,double pos_z,double pos_theta)
        {

        }
        ros::NodeHandle nh_;
        Log_executor::Ptr log_exe_ptr_;
        std::stringstream str;
};