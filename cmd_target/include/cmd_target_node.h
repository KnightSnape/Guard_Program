#pragma once

#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/PointStamped.h>
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


    private:

        int init_argc;
        char **init_argv;
        ros::NodeHandle nh;
        ros::Subscriber odom_sub;
        ros::Publisher way_pub;
        geometry_msgs::PointStamped msg;


};

}