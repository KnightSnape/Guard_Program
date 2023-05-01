#pragma once

#include<ros/ros.h>
#include<sensor_msgs/Joy.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/PointStamped.h>
#include<thread>
#include<memory>


class cmd_solver
{
    public:
        cmd_solver(ros::NodeHandle& nh);
        void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
        void main_process();

        
        ros::Subscriber odom_subscriber;
        ros::Publisher Joy_publisher;
        ros::Publisher Target_publisher;
    private:
        float vehicle_z;
        std::thread main_thread;

};  