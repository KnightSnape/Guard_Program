#include"chassis_executor.hpp"

Chassis_executor::Chassis_executor()
{
    std::string topic_path = ros::package::getPath("armor_decision") + "/config/topic.yaml";
    YAML::Node Config = YAML::LoadFile(topic_path);

    std::string navigation_target_topic = Config["Advertise"]["navigation_target_topic"].as<std::string>();
    std::string joy_topic = Config["Advertise"]["joy_topic"].as<std::string>();
    std::string cmd_twist_topic = Config["Advertise"]["cmd_twist_topic"].as<std::string>();
    std::string use_navigation_topic = Config["Advertise"]["use_navigation_topic"].as<std::string>();

    navigation_point_pub = nh_.advertise<geometry_msgs::PointStamped>(navigation_target_topic,5);
    joy_pub = nh_.advertise<sensor_msgs::Joy>(joy_topic,5);
    twist_vector_pub = nh_.advertise<geometry_msgs::TwistStamped>(cmd_twist_topic,5);
    use_navigation_pub = nh_.advertise<std_msgs::Bool>(use_navigation_topic,1);
}

void Chassis_executor::pub_nav_point(geometry_msgs::PointStamped msg)
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

    joy.header.frame_id = "waypoint_tool";
    joy.header.stamp = ros::Time::now();
    joy_pub.publish(joy);

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "body";
    navigation_point_pub.publish(msg);

    std_msgs::Bool navigating;
    navigating.data = true;
    use_navigation_pub.publish(navigating);

    ROS_DEBUG("navigation target publish done");
}

void Chassis_executor::pub_stop_signal()
{
    std_msgs::Bool navigation_stop;
    navigation_stop.data = false;
    use_navigation_pub.publish(navigation_stop);
}

void Chassis_executor::pub_twist(geometry_msgs::TwistStamped msg)
{
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "camera_init";
    twist_vector_pub.publish(msg);

    std_msgs::Bool navigating;
    navigating.data = false;
    use_navigation_pub.publish(navigating);

    ROS_DEBUG("twist target publish done");
}