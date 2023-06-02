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
#include<thread>
#include<mutex>

void odometry_callback(const nav_msgs::Odometry::ConstPtr msg)
{

}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"navigation_node");
    ros::NodeHandle nh;
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry",100000,odometry_callback);
    ros::spin();
    return 0;
}