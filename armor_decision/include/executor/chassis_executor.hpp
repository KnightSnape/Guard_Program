#pragma once
#include<ros/ros.h>
#include"log_executor.hpp"
#include"gimbal_executor.hpp"
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/TwistStamped.h>
#include<std_msgs/Bool.h>
#include<sensor_msgs/Joy.h>


class Chassis_executor
{
    public:
        typedef std::shared_ptr<Chassis_executor> Ptr;
        Chassis_executor();
        void pub_nav_point(geometry_msgs::PointStamped msg);
        void pub_twist(geometry_msgs::TwistStamped msg);
        void pub_stop_signal();

        ros::NodeHandle nh_;
        Log_executor::Ptr log_exe_ptr_;
        std::stringstream str;
        ros::Publisher use_navigation_pub;
        ros::Publisher navigation_point_pub;
        ros::Publisher joy_pub;
        ros::Publisher twist_vector_pub;
};