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

    self_pos = Eigen::Vector3d::Zero();
    target_point = Eigen::Vector3d::Zero();

    odom_sub = nh.subscribe(odom_sub_topic,5,&navigation::odom_callback,this);
    signal_sub = nh.subscribe(signal_sub_topic,5,&navigation::signal_callback,this);
    cmd_sub = nh.subscribe(navigation_sub_topic,5,&navigation::cmd_vel_callback,this);
    twist_pub = nh.advertise<geometry_msgs::Twist>(cmd_pub_topic,5);

    is_just_rotate = false;

    ROS_DEBUG("Init done");

    main_process();

    ros::spin();
}

void navigation::odom_callback(const nav_msgs::Odometry::ConstPtr msg)
{
    this->odom = (*msg);
}

void navigation::signal_callback(const std_msgs::Bool::ConstPtr msg)
{
    this->is_navigating = msg->data;
}

void navigation::cmd_vel_callback(const geometry_msgs::PointStamped::ConstPtr msg)
{
    this->point = (*msg);
    start_navigation = true;
}

void navigation::odom_to_vec3()
{
    self_pos.x() = odom.pose.pose.position.x;
    self_pos.y() = odom.pose.pose.position.y;
    self_pos.z() = odom.pose.pose.position.z;
    self_q.w() = odom.pose.pose.orientation.w;
    self_q.x() = odom.pose.pose.orientation.x;
    self_q.y() = odom.pose.pose.orientation.y;
    self_q.z() = odom.pose.pose.orientation.z;
    cal_odom_yaw();
}

void navigation::target_to_vec3()
{
    target_point.x() = point.point.x;
    target_point.y() = point.point.y;
    target_point.z() = point.point.z;
    cal_target_yaw();
}

void navigation::cal_odom_yaw()
{
    Eigen::Vector3d eularAngle = self_q.matrix().eulerAngles(2,1,0);
    yaw_now = eularAngle[2];
}

void navigation::cal_target_yaw()
{
    Eigen::Vector3d speed_vector = target_point - self_pos;
    Eigen::Vector2d speed_vector_2d{speed_vector.x(),speed_vector_2d.y()};
    Eigen::Vector2d x_axis{1.0,0};
    double cos_value;
    if(speed_vector_2d.norm() == 0)
        cos_value = 0;
    else
        cos_value = (speed_vector_2d.x() * x_axis.x() + speed_vector_2d.y() * x_axis.y()) / (speed_vector_2d.norm() * x_axis.norm());
    yaw_target = asin(cos_value);
    ROS_DEBUG("yaw target = %f",yaw_target);
}   

void navigation::pub_twist(const Eigen::Vector3d& twist_eigen)
{
    geometry_msgs::TwistStamped twist;
    twist.twist.linear.x = twist_eigen.x();
    twist.twist.linear.y = twist_eigen.y();
    twist.twist.linear.z = twist_eigen.z();
    twist.twist.angular.x = 0.0;
    twist.twist.angular.y = 0.0;
    twist.twist.angular.z = final_rotate_yaw;
    twist.header.stamp = ros::Time::now();
    twist.header.frame_id = "camera_init";
    twist_pub.publish(twist);
    ROS_DEBUG("publish twist done");
}
void navigation::main_process()
{
    while(1)
    {
        odom_to_vec3();
        target_to_vec3();
        if(!start_navigation)
            continue;
        if(!is_navigating)
            continue;
        Eigen::Vector3d final_speed;
        double delta_yaw = yaw_now - yaw_target;
        if(delta_yaw > 3.0 / 180 * M_PI || delta_yaw < -3.0 / 180 * M_PI)
        {
            if(delta_yaw > 3.0 / 180 * M_PI)
            {
                final_rotate_yaw = 3.0 / 180 * M_PI;
            }
            else
            {
                final_rotate_yaw = -3.0 / 180 * M_PI;
            }
            is_just_rotate = true;
        }

        if(!is_just_rotate)
        {
            double distance = (target_point - self_pos).norm();
            if(distance > max_length)
            {
                final_speed.x() = max_speed;
                final_speed.y() = 0.0;
                final_speed.z() = 0.0;
            }
            else if(distance > min_length && distance <= max_length)
            {
                final_speed.x() = max_speed / 2;
                final_speed.y() = 0.0;
                final_speed.z() = 0.0;
            }
            else
            {
                final_speed = Eigen::Vector3d::Zero();
            }
            final_rotate_yaw = 0.0;
        }

        ROS_DEBUG("Final speed is %f,%f,%f",final_speed.x(),final_speed.y(),final_speed.z());
        pub_twist(final_speed);
        is_just_rotate = false;
    }


}