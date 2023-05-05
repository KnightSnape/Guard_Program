#include"autoaim_executor.hpp"

void AutoAim_executor::initParam(ros::NodeHandle &nh)
{
    std::string topic_path = ros::package::getPath("armor_decision") + "/config/topic.yaml";
    YAML::Node Config = YAML::LoadFile(topic_path);

    std::string autoaim_state_topic = Config["Advertise"]["autoaim_state_topic"].as<std::string>();

    status_publisher = nh.advertise<guard_msgs::AutoaimState>(autoaim_state_topic,5);
}


void AutoAim_executor::pub_autoaim_state(int state,int target_id)
{
    guard_msgs::AutoaimState status_msg;
    status_msg.autoaim_state = (uint8_t)state;

    status_msg.target_id = (uint8_t)target_id;

    status_msg.header.frame_id = "camera_init";
    status_msg.header.stamp = ros::Time::now();
}
