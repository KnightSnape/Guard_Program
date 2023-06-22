#pragma once

#include<ros/ros.h>
#include<ros/package.h>
#include<nav_msgs/Odometry.h>
#include<Eigen/Eigen>
#include<geometry_msgs/PointStamped.h>
#include"gary_msgs/ClientCommand.h"
#include<QtGui>
#include<QThread>
#include<QMainWindow>
#include<QApplication>
#include<QMutex>
#include<QListWidgetItem>
#include<string>

namespace qt5_solver
{

class QNode : public QThread
{   
    public:
        typedef std::shared_ptr<QNode> Ptr;
        QNode(int argc,char **argv);
        ~QNode();
        void run() override;
        void get_final_navigaton(float X,float Y);
        void publish_command(Eigen::Vector3d target_pos,char command_id);
        

    private:

        int init_argc;
        char **init_argv;
        ros::NodeHandle nh;
        ros::Subscriber odom_sub;
        ros::Publisher way_pub;
        ros::Publisher command_pub;
        geometry_msgs::PointStamped msg;
        gary_msgs::ClientCommand command;


};

}