#include<string>
#include<ros/ros.h>
#include<ros/package.h>
#include<Eigen/Eigen>
#include<yaml-cpp/yaml.h>
#include"std_msgs/Bool.h"
#include"nav_msgs/Odometry.h"
#include"geometry_msgs/PointStamped.h"
#include"geometry_msgs/TwistStamped.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include<memory>
#include<thread>
#include<mutex>

nav_msgs::Odometry odom;
bool is_odom_sub;
bool is_signal = false;
Eigen::Vector3d self_pos;
Eigen::Vector3d target_pos;
Eigen::Quaterniond self_q;

void odometry_callback(const nav_msgs::Odometry::ConstPtr msg)
{
    odom = *msg;
    is_odom_sub = true;
}

void signal_callback(const std_msgs::Bool::ConstPtr msg)
{
    is_signal = msg->data;
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"navigation_node");
    ros::NodeHandle nh;
    is_odom_sub = false;
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry",100000,odometry_callback);
    ros::Subscriber signal_sub = nh.subscribe<std_msgs::Bool>("/use_navigation",5,signal_callback);

    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel",5);
    while(ros::ok())
    {
        if(is_odom_sub)
        {
            self_pos.x() = odom.pose.pose.position.x;
            self_pos.y() = odom.pose.pose.position.y;
            self_pos.z() = odom.pose.pose.position.z;

            self_q.w() = odom.pose.pose.orientation.w;
            self_q.x() = odom.pose.pose.orientation.x;
            self_q.y() = odom.pose.pose.orientation.y;
            self_q.z() = odom.pose.pose.orientation.z;

            double roll;
            double pitch;
            double yaw;

            geometry_msgs::Quaternion geoQuat = odom.pose.pose.orientation;
            tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

            target_pos << 1.0, 0, 0;
            Eigen::Vector2d target_pos_2d{target_pos[0],target_pos[1]};
            Eigen::Vector2d self_pos_2d{self_pos[0],self_pos[1]};
            Eigen::Vector2d delta_pos = target_pos_2d - self_pos_2d;
            Eigen::Vector2d x_pos{1.0,0};
            double norm = delta_pos.norm();
            if(norm == 0)
                continue;
            double cos_angle = (delta_pos.x() * x_pos.x() + delta_pos.y() * x_pos.y()) / norm;
            double angle = acos(cos_angle);
            double delta_angle = yaw - angle;
            bool use_final_angle = false;
            double final_angle = 0.0;
            if(delta_angle > 8.0 / 180 * M_PI)
            {
                final_angle = 5.0 / 180 * M_PI;
                use_final_angle = true;
            }
            if(delta_angle < -8.0 / 180 * M_PI)
            {
                final_angle = -5.0 / 180 * M_PI;
                use_final_angle = true;
            }
            Eigen::Vector3d final_twist = Eigen::Vector3d::Zero();
            if(!use_final_angle)
            {
                double final_speed = 0;
                if(norm > 3)
                    final_speed = 1.0;
                else if(norm > 0.2 && norm <= 3)
                    final_speed = 0.5;
                else 
                    final_speed = 0.1;
                final_twist.x() = final_speed;
                final_twist.y() = 0.0;
                final_twist.z() = 0.0;
            }

            geometry_msgs::TwistStamped twist;
            twist.header.frame_id = "camera_init";
            twist.header.stamp = ros::Time::now();
            twist.twist.linear.x = final_twist.x();
            twist.twist.linear.y = final_twist.y();
            twist.twist.linear.z = final_twist.z();
            twist.twist.angular.x = 0.0;
            twist.twist.angular.y = 0.0;
            twist.twist.angular.z = final_angle;

            cmd_pub.publish(twist);
            is_odom_sub = false;
        }

        ros::spinOnce();
    }
    return 0;
}