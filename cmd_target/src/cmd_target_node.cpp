#include"cmd_target_node.h"

namespace qt5_solver
{

QNode::QNode(int argc,char **argv):
init_argc(argc),
init_argv(argv)
{
    way_pub = nh.advertise<geometry_msgs::PointStamped>("/way_point",5);
}

QNode::~QNode()
{
    ros::shutdown();
}

void QNode::run()
{
    ros::spin();
}

void QNode::get_final_navigaton(float X,float Y)
{
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "camera_init";
    msg.point.x = X;
    msg.point.y = Y;
    msg.point.z = 0.0;
    way_pub.publish(msg);
    ROS_DEBUG("publish done");
}




}