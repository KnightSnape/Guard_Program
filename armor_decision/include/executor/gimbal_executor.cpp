#include"gimbal_executor.hpp"

void Gimbal_executor::initParams(ros::NodeHandle &nh)
{
    std::string topic_path = ros::package::getPath("armor_decision") + "/config/topic.yaml";
    std::string config_path = ros::package::getPath("armor_decision") + "/config/config.yaml";

    YAML::Node Config = YAML::LoadFile(config_path);
    YAML::Node Topic = YAML::LoadFile(topic_path);

    pitch_upper_threshold = Config["pitch_upper_threshold"].as<float>();
    pitch_lower_threshold = Config["pitch_lower_threshold"].as<float>();
    yaw_rotate_speed = Config["yaw_work_speed"].as<float>();
    pitch_rotate_speed = Config["pitch_work_speed"].as<float>();
    rotate_freq = Config["rotate_freq"].as<float>();

    std::string pitch_set_topic = Topic["Advertise"]["pitch_set_topic"].as<std::string>();
    std::string yaw_set_topic = Topic["Advertise"]["yaw_set_topic"].as<std::string>();

    gimbal_pitch_now = 0;
    gimbal_yaw_now = 0;

    pitch_sign = true;
    yaw_sign = true;

    pitch_set_pub = nh.advertise<std_msgs::Float64>(pitch_set_topic,5);
    yaw_set_pub = nh.advertise<std_msgs::Float64>(yaw_set_topic,5);
}

void Gimbal_executor::get_pitch_now(gary_msgs::DualLoopPIDWithFilter msg)
{
    gimbal_pitch_now = msg.outer_feedback;
}

void Gimbal_executor::get_yaw_now(gary_msgs::DualLoopPIDWithFilter msg)
{
    gimbal_yaw_now = msg.outer_feedback;
}

void Gimbal_executor::operate_state(int state)
{
    if(state == 0)
        return;
    if(state == 1)
    {
        
    }
    //LOW_SPEED
    if(state == 2)
    {
        yaw_upper_threshold = 40.0 * PI / 180;
        yaw_lower_threshold = -40.0 * PI / 180;
        float gimbal_pitch_delta = (PI * 2.0) * pitch_rotate_speed / rotate_freq;
        if(pitch_sign)
        {
            gimbal_pitch_now = gimbal_pitch_now + gimbal_pitch_delta;
            if(gimbal_pitch_now + gimbal_pitch_delta > pitch_upper_threshold)
            {
                pitch_sign = false;
            }
        }
        else
        {
            gimbal_pitch_now = gimbal_pitch_now - gimbal_pitch_delta;
            if(gimbal_pitch_now - gimbal_pitch_delta < pitch_lower_threshold)
            {
                pitch_sign = true;
            }
        }

        float gimbal_yaw_delta = (PI * 2.0) * yaw_rotate_speed / rotate_freq;
        if(yaw_sign)
        {
            gimbal_yaw_now = gimbal_yaw_now + gimbal_yaw_delta;
            if(gimbal_yaw_now + gimbal_yaw_delta > yaw_upper_threshold)
            {
                yaw_sign = false;
            }
        }
        else
        {
            gimbal_yaw_now = gimbal_yaw_now - gimbal_yaw_delta;
            if(gimbal_yaw_now - gimbal_yaw_delta < yaw_lower_threshold)
            {
                yaw_sign = true;
            }
        }
    }
    //HIGH_SPEED
    if(state == 3)
    {
        yaw_upper_threshold = 70.0 * PI / 180;
        yaw_lower_threshold = -70.0 * PI / 180;
        float gimbal_pitch_delta = (PI * 2.0) * pitch_rotate_speed / rotate_freq;
        if(pitch_sign)
        {
            gimbal_pitch_now = gimbal_pitch_now + gimbal_pitch_delta;
            if(gimbal_pitch_now + gimbal_pitch_delta > pitch_upper_threshold)
            {
                pitch_sign = false;
            }
        }
        else
        {
            gimbal_pitch_now = gimbal_pitch_now - gimbal_pitch_delta;
            if(gimbal_pitch_now - gimbal_pitch_delta < pitch_lower_threshold)
            {
                pitch_sign = true;
            }
        }

        float gimbal_yaw_delta = (PI * 2.0) * yaw_rotate_speed * 2 / rotate_freq;
        if(yaw_sign)
        {
            gimbal_yaw_now = gimbal_yaw_now + gimbal_yaw_delta;
            if(gimbal_yaw_now + gimbal_yaw_delta > yaw_upper_threshold)
            {
                yaw_sign = false;
            }
        }
        else
        {
            gimbal_yaw_now = gimbal_yaw_now - gimbal_yaw_delta;
            if(gimbal_yaw_now - gimbal_yaw_delta < yaw_lower_threshold)
            {
                yaw_sign = true;
            }
        }
    }
    //SCANNING
    if(state == 4)
    {
        float gimbal_yaw_delta = (PI * 2.0) * yaw_rotate_speed / rotate_freq;
        //等会儿考虑pitch
        float gimbal_pitch_delta = (PI * 2.0) * pitch_rotate_speed / rotate_freq;
        gimbal_yaw_now = gimbal_yaw_now + gimbal_yaw_delta;

        if(pitch_sign)
        {
            gimbal_pitch_now = gimbal_pitch_now + gimbal_pitch_delta;
            if(gimbal_pitch_now + gimbal_pitch_delta > pitch_upper_threshold)
            {
                pitch_sign = false;
            }
        }
        else
        {
            gimbal_pitch_now = gimbal_pitch_now - gimbal_pitch_delta;
            if(gimbal_pitch_now - gimbal_pitch_delta < pitch_lower_threshold)
            {
                pitch_sign = true;
            }
        }


    }
    std_msgs::Float64 pitch_msg;
    std_msgs::Float64 yaw_msg;
    pitch_msg.data = gimbal_pitch_now;
    yaw_msg.data = gimbal_yaw_now;

}