
#include"cmd_target_node.h"

namespace qt5_solver
{

QNode::QNode(int argc,char **argv):
init_argc(argc),
init_argv(argv)
{
    way_pub = nh.advertise<geometry_msgs::PointStamped>("/way_point",5);
    command_pub = nh.advertise<gary_msgs::ClientCommand>("/referee/client_command",5);
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

void QNode::publish_command(Eigen::Vector3d target_pos,char command_id)
{
    command.target_robot_id = (uint16_t)0;
    command.target_position_x = (float)target_pos.x();
    command.target_position_y = (float)target_pos.y();
    command.target_position_z = (float)target_pos.z();
    command.keyboard_key_pressed = (uint8_t)command_id;
    command.header.frame_id = "camera_init";
    command.header.stamp = ros::Time::now();
    command_pub.publish(command);
    ROS_DEBUG("command publish done");
}



}