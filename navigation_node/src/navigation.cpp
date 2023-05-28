#include"navigation.hpp"

navigation::navigation()
{
    start_navigation = false;
    is_navigating = false;
    
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    std::string path = ros::package::getPath("navigation_node") + "/config/";
    std::string topic_path = path + "topic.yaml";
    YAML::Node Config = YAML::LoadFile(topic_path);

    std::string odom_sub_topic = Config["Subscribe"]["odomtry"].as<std::string>();
    std::string navigation_sub_topic = Config["Subscribe"]["navigation"].as<std::string>();
    std::string signal_sub_topic = Config["Subscribe"]["signal"].as<std::string>();
    std::string cmd_pub_topic = Config["Advertise"]["cmd"].as<std::string>();

    std::string config_path = path + "config.yaml";
    
    Config = YAML::LoadFile(config_path);
    max_speed = Config["max_speed"].as<double>();
    max_length = Config["max_length"].as<double>();
    min_length = Config["min_length"].as<double>();

    odom_sub = nh.subscribe(odom_sub_topic,5,&navigation::odom_callback,this);
    signal_sub = nh.subscribe(signal_sub_topic,5,&navigation::signal_callback,this);
    cmd_sub = nh.subscribe(navigation_sub_topic,5,&navigation::cmd_vel_callback,this);
    twist_pub = nh.advertise<geometry_msgs::Twist>(cmd_pub_topic,5);

    ROS_DEBUG("Init done");
}

void navigation::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    this->odom = (*msg);
}

void navigation::signal_callback(const std_msgs::Bool::ConstPtr& msg)
{
    this->is_navigating = msg->data;
}

void navigation::cmd_vel_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    this->point = (*msg);
    start_navigation = true;
}

void navigation::odom_to_vec3()
{
    self_pos.x() = odom.pose.pose.position.x;
    self_pos.y() = odom.pose.pose.position.y;
    self_pos.z() = odom.pose.pose.position.z;
}

void navigation::target_to_vec3()
{
    target_point.x() = point.point.x;
    target_point.y() = point.point.y;
    target_point.z() = point.point.z;
}

void navigation::pub_twist(const Eigen::Vector3d& twist_eigen)
{
    geometry_msgs::TwistStamped twist;
    twist.twist.linear.x = twist_eigen.x();
    twist.twist.linear.y = twist_eigen.y();
    twist.twist.linear.z = twist_eigen.z();
    twist.twist.angular.x = 0.0;
    twist.twist.angular.y = 0.0;
    twist.twist.angular.z = 0.0;
    twist.header.stamp = ros::Time::now();
    twist.header.frame_id = "camera_init";
    twist_pub.publish(twist);
    ROS_DEBUG("publish twist done");
}

void navigation::main_process()
{
    while(1)
    {
        if(!start_navigation)
            continue;
        if(!is_navigating)
            continue;
        odom_to_vec3();
        target_to_vec3();
        Eigen::Vector3d speed_vector = target_point - self_pos;
        speed_vector.z() = 0.0;
        Eigen::Vector3d e_speed = speed_vector / speed_vector.norm();
        Eigen::Vector3d final_speed;
        float length = speed_vector.norm();
        if(length > max_length)
            final_speed = e_speed * max_speed;
        else if(length >= min_length && length <= max_length)
            final_speed = e_speed * max_speed / 3.0;
        else
            final_speed = Eigen::Vector3d::Zero();
        ROS_DEBUG("Final speed is %f,%f,%f",final_speed.x(),final_speed.y(),final_speed.z());
        pub_twist(final_speed);
    }


}