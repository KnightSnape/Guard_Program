#include"gimbal_executor.hpp"

Gimbal_executor::Gimbal_executor()
{
    std::string topic_path = ros::package::getPath("armor_decision") + "/config/topic.yaml";
    std::string config_path = ros::package::getPath("armor_decision") + "/config/config.yaml";

    YAML::Node Config = YAML::LoadFile(config_path);
    YAML::Node Topic = YAML::LoadFile(topic_path);

    pitch_upper_threshold = Config["pitch_upper_threshold"].as<double>();
    pitch_lower_threshold = Config["pitch_lower_threshold"].as<double>();
    yaw_rotate_speed = Config["yaw_work_speed"].as<double>();
    pitch_rotate_speed = Config["pitch_rotate_speed"].as<double>();
    rotate_freq = Config["rotate_freq"].as<double>();
    k_autoaim = Config["k_autoaim"].as<double>();
    x_offset = Config["x_offset"].as<double>();
    y_offset = Config["y_offset"].as<double>();

    std::string pitch_set_topic = Topic["Advertise"]["pitch_set_topic"].as<std::string>();
    std::string yaw_set_topic = Topic["Advertise"]["yaw_set_topic"].as<std::string>();

    pitch_sign = true;
    yaw_sign = true;

    pitch_set_pub = nh.advertise<std_msgs::Float64>(pitch_set_topic,1000);
    yaw_set_pub = nh.advertise<std_msgs::Float64>(yaw_set_topic,1000);

}

void Gimbal_executor::get_pitch_now(gary_msgs::DualLoopPIDWithFilter msg)
{
    gimbal_pitch_now = msg.outer_feedback;
}

void Gimbal_executor::get_yaw_now(gary_msgs::DualLoopPIDWithFilter msg)
{
    gimbal_yaw_now = msg.outer_feedback;
}

void Gimbal_executor::get_autoaim_target(gary_msgs::AutoAIM msg)
{
    this->pitch_autoaim = msg.pitch;
    this->yaw_autoaim = msg.yaw;
}

void Gimbal_executor::operate_state(Gimbal_Mode state)
{
    if(state == Gimbal_Mode::STEADY)
    {
        
    }
    if(state == Gimbal_Mode::WARN_CAPTAIN)
    {
        
    }
    //LOW_SPEED
    if(state == Gimbal_Mode::LOW_SPEED)
    {
        yaw_upper_threshold = 40.0;
        yaw_lower_threshold = -40.0;
        double gimbal_pitch_delta = (PI * 2.0) * pitch_rotate_speed / rotate_freq;
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

        double gimbal_yaw_delta = (PI * 2.0) * yaw_rotate_speed / rotate_freq;
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
    if(state == Gimbal_Mode::HIGH_SPEED)
    {
        yaw_upper_threshold = 70.0;
        yaw_lower_threshold = -70.0;
        double gimbal_pitch_delta = (PI * 2.0) * pitch_rotate_speed / rotate_freq;
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

        double gimbal_yaw_delta = (PI * 2.0) * yaw_rotate_speed * 2 / rotate_freq;
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
    if(state == Gimbal_Mode::SCANNING)
    {
        double gimbal_yaw_delta = (PI * 2.0) * yaw_rotate_speed / rotate_freq;
        //等会儿考虑pitch
        double gimbal_pitch_delta = (PI * 2.0) * pitch_rotate_speed / rotate_freq;
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
    if(state == Gimbal_Mode::FOLLOW_AUTOAIM)
    {
        gimbal_pitch_now = gimbal_pitch_now - pitch_autoaim * k_autoaim + x_offset;
        gimbal_yaw_now = gimbal_yaw_now - yaw_autoaim * k_autoaim + y_offset;
    }
    pitch_msg.data = gimbal_pitch_now;
    yaw_msg.data = gimbal_yaw_now;
    pitch_set_pub.publish(pitch_msg);
    yaw_set_pub.publish(yaw_msg);
}