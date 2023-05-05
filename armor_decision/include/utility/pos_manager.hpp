#pragma once
#include<geometry_msgs/PointStamped.h>
#include"utility.hpp"

class Pos_Manager
{
    public:
        Pos_Manager();
        Eigen::Vector2i inverse_point(const Eigen::Vector2i &p);
        Eigen::Vector2i refresh_point(const Eigen::Vector2i &p);
        Eigen::Vector2i world_to_map(const Eigen::Vector3d &pw);
        Eigen::Vector3d map_to_world(const Eigen::Vector2i &pm);
        geometry_msgs::PointStamped transfer_to_msg(const Eigen::Vector3d &pw);
        void set_guard_world_pos(const Eigen::Vector3d &point);
        void set_guard_world_pos(const nav_msgs::Odometry &odom);
        void set_guard_rotate_q(const Eigen::Quaterniond &q);
        void set_guard_rotate_q(const nav_msgs::Odometry &odom);
        void set_pos_x_zero(int pos_x_zero);
        void set_pos_y_zero(int pos_y_zero);
        float get_yaw();
        Eigen::Vector3d get_guard_world_pos();
        Eigen::Quaterniond get_guard_rotate_q();
        void rotate_angle(float angle);
        void speed_process(float &twist_x,float &twist_y,int mode,float &yaw_angle);
        

    private:
        Eigen::Vector3d guard_world_pos_now;
        Eigen::Quaterniond guard_rotate_q;
        float max_rotate_angle;
        int pos_x_zero;
        int pos_y_zero;
        inline float getsign(float a,float b)
        {
            return ((a-b)>0)?1:-1;
        }
        

};