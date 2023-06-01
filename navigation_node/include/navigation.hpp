#pragma once

#include<string>
#include<ros/ros.h>
#include<ros/package.h>
#include<Eigen/Eigen>
#include<yaml-cpp/yaml.h>
#include"std_msgs/Bool.h"
#include"nav_msgs/Odometry.h"
#include"geometry_msgs/PointStamped.h"
#include"geometry_msgs/TwistStamped.h"
#include<memory>

class navigation
{
    public:
        typedef std::shared_ptr<navigation> Ptr;
        navigation();

        void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
        void signal_callback(const std_msgs::Bool::ConstPtr& msg);
        void cmd_vel_callback(const geometry_msgs::PointStamped::ConstPtr& msg);
        void odom_to_vec3();
        void target_to_vec3();
        void cal_odom_yaw();
        void cal_target_yaw();
        void pub_twist(const Eigen::Vector3d& twist_eigen);
        void main_process();

    private:
        ros::NodeHandle nh;

        ros::Subscriber odom_sub;
        ros::Subscriber signal_sub;
        ros::Subscriber cmd_sub;

        ros::Publisher twist_pub;

        nav_msgs::Odometry odom;
        geometry_msgs::PointStamped point;
        Eigen::Vector3d target_point;
        Eigen::Vector3d self_pos;
        Eigen::Quaterniond self_q;
        Eigen::Quaterniond final_q;

        double yaw_now;
        double yaw_target;
        double final_rotate_yaw;

        bool is_navigating;
        bool start_navigation;
        bool is_just_rotate;

        double max_speed;
        double max_length;
        double min_length;



};