#include"cmd_target.hpp"

cmd_solver::cmd_solver(ros::NodeHandle& nh)
{
    this->vehicle_z = 0;
    odom_subscriber = nh.subscribe("/Odometry",5,&cmd_solver::odom_callback,this);
    Target_publisher = nh.advertise<geometry_msgs::PointStamped>("/way_point",5);
    Joy_publisher = nh.advertise<sensor_msgs::Joy>("/joy",5);

    main_thread = std::thread(&cmd_solver::main_process,this);
}

void cmd_solver::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    this->vehicle_z = msg->pose.pose.position.z;
}

//假定一个循环
void cmd_solver::main_process()
{

    sensor_msgs::Joy joy;
    joy.axes.push_back(0);
    joy.axes.push_back(0);
    joy.axes.push_back(-1.0);
    joy.axes.push_back(0);
    joy.axes.push_back(1.0);
    joy.axes.push_back(1.0);
    joy.axes.push_back(0);
    joy.axes.push_back(0);

    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(1);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);

    joy.header.stamp = ros::Time::now();
    joy.header.frame_id = "waypoint_tool";
    Joy_publisher.publish(joy);

    geometry_msgs::PointStamped waypoint;
    waypoint.header.frame_id = "body";
    waypoint.header.stamp = joy.header.stamp;
    waypoint.point.x = 0;
    waypoint.point.y = 0.2;
    waypoint.point.z = vehicle_z;

    Target_publisher.publish(waypoint);

    usleep(10000);
    Target_publisher.publish(waypoint);
}