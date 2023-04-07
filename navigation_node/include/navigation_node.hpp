#pragma once

#include<iostream>
#include<chrono>
#include<ros/ros.h>
#include<memory>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<geometry_msgs/TransformStamped.h>
#include<sensor_msgs/Imu.h>
#include<sensor_msgs/PointCloud2.h>
#include<nav_msgs/Odometry.h>
#include<nav_msgs/Path.h>
#include<sensor_msgs/LaserScan.h>
#include<thread>
#include<mutex>
#include<yaml-cpp/yaml.h>
#include<ros/package.h>
#include<tf2/LinearMath/Quaternion.h>
#include<tf2_ros/transform_broadcaster.h>
#include"mapping_2d_tf_solver.hpp"
#include"pointcloud_2d_solver.hpp"
#include"parameter.h"

namespace navigation
{

class Navigation_Solver
{
    public:

        Navigation_Solver(int argc,char **argv)
        {
            is_odometry_received = false;
            is_path_received = false;
            is_point_cloud2_received = false;

            std::string config_path = ros::package::getPath("navigation_node") + "/config/config.yaml";
            read_param(config_path);

            point_cloud_2d_solver = std::make_shared<PointCloud_2d_Solver>();
            YAML::Node Config = YAML::LoadFile(config_path);

            int is_2d_solver = Config["is_2d_solver"].as<int>();
            
            Odometry_sub = nh.subscribe("/Odometry",100000,&Navigation_Solver::Odometry_Lidar_Callback,this);
            PointCloud2_sub = nh.subscribe("/cloud_registered",100000,&Navigation_Solver::PointCloud_Callback,this);

            Odometry_pub = nh.advertise<nav_msgs::Odometry>("/Odometry_2d",80000);
            down_sample_PointCloud2_pub = nh.advertise<sensor_msgs::PointCloud2>("/checker_pointcloud2",80000);
            Initial_Pos = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",80000);
            grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("/grid_map",80000);

            initPosepub();

            if(is_2d_solver)
            {
                navigation_thread = std::thread(&Navigation_Solver::navigation_node_solver_2d,this);
            }
            else
            {
                navigation_thread = std::thread(&Navigation_Solver::navigation_node_solver_3d,this);
            }
        }
    private:
        ros::Subscriber Odometry_sub;
        ros::Subscriber Path_sub;
        ros::Subscriber PointCloud2_sub;

        ros::Publisher Odometry_pub;
        ros::Publisher down_sample_PointCloud2_pub;
        ros::Publisher LaserScan_pub;
        ros::Publisher Initial_Pos;
        ros::Publisher grid_pub;

        std::mutex odometry_lidar_mutex;
        std::mutex path_lidar_mutex;
        std::mutex pointcloud2_mutex;

        std::thread navigation_thread;

        nav_msgs::Odometry odometry_lidar;
        nav_msgs::Path path;
        sensor_msgs::PointCloud2 point_cloud2;
        sensor_msgs::LaserScan laser_scan_msg;

        std::shared_ptr<PointCloud_2d_Solver> point_cloud_2d_solver;
        TF tf_2dmap;

        tf2_ros::TransformBroadcaster tf_broadcaster;

        bool is_odometry_received;
        bool is_path_received;
        bool is_point_cloud2_received;

        void Odometry_Lidar_Callback(const nav_msgs::Odometry::ConstPtr& msg)
        {
            odometry_lidar_mutex.lock();
            is_odometry_received = true;
            odometry_lidar.pose.pose.position.x = msg->pose.pose.position.x;
            odometry_lidar.pose.pose.position.y = msg->pose.pose.position.y;
            odometry_lidar.pose.pose.position.z = 0;
            odometry_lidar.child_frame_id = msg->child_frame_id;
            odometry_lidar.pose.pose.orientation = msg->pose.pose.orientation;
            odometry_lidar.twist.twist.linear.x = msg->twist.twist.linear.x;
            odometry_lidar.twist.twist.linear.y = msg->twist.twist.linear.y;
            odometry_lidar.twist.twist.linear.z = 0;
            odometry_lidar.twist.twist.angular = msg->twist.twist.angular;
            odometry_lidar_mutex.unlock();
        }

        void Path_Lidar_Callback(const nav_msgs::Path::ConstPtr& msg)
        {
            
        }

        void PointCloud_Callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
        {
            pointcloud2_mutex.lock();
            point_cloud2.fields = msg->fields;
            point_cloud2.width = msg->width;
            point_cloud2.height = msg->height;
            point_cloud2.point_step = msg->point_step;
            point_cloud2.row_step = msg->row_step;
            point_cloud2.is_bigendian = msg->is_bigendian;
            point_cloud2.data = msg->data;
            point_cloud2.is_dense = msg->is_dense;
            is_point_cloud2_received = true;
            pointcloud2_mutex.unlock();
        }

        void navigation_node_solver_2d()
        {
            while(1)
            {
                if(is_point_cloud2_received)
                {
                    point_cloud_2d_solver->get_pointcloud(point_cloud2);
                    nav_msgs::OccupancyGrid grid_msg = point_cloud_2d_solver->PointCloud2ToGrid();
                    sensor_msgs::PointCloud2 down_sample_PointCloud2_msg = point_cloud_2d_solver->get_downsampled_pointcloud2();
                    grid_msg.header.stamp = ros::Time::now();
                    grid_msg.header.frame_id = "camera_init";
                    grid_pub.publish(grid_msg);
                    down_sample_PointCloud2_msg.header.stamp = ros::Time::now();
                    down_sample_PointCloud2_msg.header.frame_id = "camera_init";
                    down_sample_PointCloud2_pub.publish(down_sample_PointCloud2_msg);

                }
                is_point_cloud2_received = false;
                if(is_odometry_received)
                {
                    odometry_lidar.header.stamp = ros::Time::now();
                    odometry_lidar.header.frame_id = "camera_init";
                    Odometry_pub.publish(odometry_lidar);
                }
                is_odometry_received = false;
                geometry_msgs::TransformStamped stamped = tf_2dmap.mapping_TF_solver();
                geometry_msgs::TransformStamped stamped_base = tf_2dmap.base_link_TF_solver();
                tf_broadcaster.sendTransform(stamped);
                tf_broadcaster.sendTransform(stamped_base);
                
            }
        }

        void navigation_node_solver_3d()
        {

        }

        void initPosepub()
        {
            geometry_msgs::PoseWithCovarianceStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "camera_init";
            pose.pose.pose.position.x = 0;
            pose.pose.pose.position.y = 0;
            pose.pose.pose.position.z = 0;
            pose.pose.pose.orientation.w = 0;
            pose.pose.pose.orientation.x = 0;
            pose.pose.pose.orientation.y = 0;
            pose.pose.pose.orientation.z = 0;
            Initial_Pos.publish(pose);
        }
        ros::NodeHandle nh;
};

}

