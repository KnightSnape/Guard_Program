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
    is_initial = true;

    gimbal_yaw_set_calc.cycle = 0.0;
    gimbal_yaw_set_calc.angle = 0.0;
    range_diff = 0.0;
    yaw_saved = 0.0;
    chassis_yaw_saved = 0.0;
    pitch_saved = 0.0;
    last_gimbal_state = Gimbal_Mode::STEADY;

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
    if(is_initial)
    {   
        this->get_init_gimbal_yaw_pos();
        is_initial = false; 
    }
}

void Gimbal_executor::get_autoaim_target(gary_msgs::AutoAIM msg)
{
    this->pitch_autoaim = msg.pitch;
    this->yaw_autoaim = msg.yaw;
}

void Gimbal_executor::get_init_gimbal_yaw_pos()
{
    this->init_yaw_pos = gimbal_yaw_now;
}

void Gimbal_executor::get_target_angle(Eigen::Vector2d target_pos_world,Eigen::Vector2d self_pos_world)
{
    //Eigen::Vector2d target_pos_2d{target_pos_world.x(),target_pos_world.y()};
    //Eigen::Vector2d self_pos_2d{self_pos_world.x(),self_pos_world.y()};
    //Eigen::Vector2d delta_pos = target_pos_2d - self_pos_2d;
    //Eigen::Vector2d x_pos{1,0};

    //double dot = (delta_pos.x() * x_pos.x() + delta_pos.y() * x_pos.y());
    //double cross = (x_pos.x() * delta_pos.y() - x_pos.y() * delta_pos.x());
    //target_angle = atan2(cross,dot);
}

void Gimbal_executor::operate_state(Gimbal_Mode state, double chassis_yaw)
{
    static const std::vector<std::string> GIMBAL_MODE_NAMES = {"NO","STEADY","LOW","HIGH","SCAN","WARN","FOLLOW"};
    ROS_INFO("Gimbal: %s",GIMBAL_MODE_NAMES[(int)state].c_str());
    bool follow_chassis = false;
    bool mode_changed = (last_gimbal_state != state);
    last_gimbal_state = state;
    if(mode_changed){
        std::cout<<"mode_changed"<<std::endl;
        range_diff = 0.0;
        // chassis_yaw_saved = chassis_yaw;
    }
    if(state == Gimbal_Mode::NO_FORCE)
    {
        return;
    }
    if (state == Gimbal_Mode::SCANNING || state == Gimbal_Mode::LOW_SPEED || state == Gimbal_Mode::HIGH_SPEED)
    {
        //LOW_SPEED or HIGH_SPEED
        if(state == Gimbal_Mode::LOW_SPEED || state == Gimbal_Mode::HIGH_SPEED)
        {
            follow_chassis = true;
            if(state == Gimbal_Mode::HIGH_SPEED){
                yaw_upper_threshold = (70.0 / 180) * PI;
                yaw_lower_threshold = (-70.0 / 180) * PI;
            }
            else if(state == Gimbal_Mode::LOW_SPEED){
                yaw_upper_threshold = (40.0 / 180) * PI;
                yaw_lower_threshold = (-40.0 / 180) * PI;
            }
            double gimbal_pitch_delta = (PI * 2.0) * pitch_rotate_speed / rotate_freq;
            if(pitch_sign)
            {
                if(gimbal_pitch_set + gimbal_pitch_delta > pitch_upper_threshold / 180 * PI)
                {
                    pitch_sign = false;
                }
                gimbal_pitch_set = gimbal_pitch_set + gimbal_pitch_delta;
            }
            else
            {
                if(gimbal_pitch_set - gimbal_pitch_delta < pitch_lower_threshold / 180 * PI)
                {
                    pitch_sign = true;
                }
                gimbal_pitch_set = gimbal_pitch_set - gimbal_pitch_delta;
            }
            double gimbal_yaw_delta = (PI * 2.0) * yaw_rotate_speed / rotate_freq;
            if(yaw_sign)
            {
                gimbal_yaw_set_calc.angle += gimbal_yaw_delta;
                range_diff += gimbal_yaw_delta;
                if(range_diff + gimbal_yaw_delta > yaw_upper_threshold)
                {
                    yaw_sign = false;
                }
            }
            else
            {
                gimbal_yaw_set_calc.angle -= gimbal_yaw_delta;
                range_diff -= gimbal_yaw_delta;
                if(range_diff - gimbal_yaw_delta < yaw_lower_threshold)
                {
                    yaw_sign = true;
                }
            }
            static double last_chassis_yaw = chassis_yaw;
            double chassis_delta = chassis_yaw - last_chassis_yaw;
            if(chassis_delta < 0-PI){chassis_delta += (2*PI);} 
            if(chassis_delta > PI){chassis_delta -= (2*PI);} 
            last_chassis_yaw = chassis_yaw;
            gimbal_yaw_set_calc.angle -= chassis_delta;
        }else{
            chassis_yaw_saved = chassis_yaw;
        }
        //SCANNING
        if(state == Gimbal_Mode::SCANNING)
        {
            double gimbal_yaw_delta = (PI * 2.0) * yaw_rotate_speed / rotate_freq;
            //等会儿考虑pitch
            double gimbal_pitch_delta = (PI * 2.0) * pitch_rotate_speed / rotate_freq;
            // gimbal_yaw_set = gimbal_yaw_now + gimbal_yaw_delta;
            gimbal_yaw_set_calc.angle += gimbal_yaw_delta;

            if(pitch_sign)
            {
                if(gimbal_pitch_set + gimbal_pitch_delta > pitch_upper_threshold / 180 * PI)
                {
                    pitch_sign = false;
                }
                gimbal_pitch_set = gimbal_pitch_set + gimbal_pitch_delta;
            }
            else
            {
                if(gimbal_pitch_set - gimbal_pitch_delta < pitch_lower_threshold / 180 * PI)
                {
                    pitch_sign = true;
                }
                gimbal_pitch_set = gimbal_pitch_set - gimbal_pitch_delta;
            }
        }

        while (gimbal_yaw_set_calc.angle > (2*PI))
        {
            gimbal_yaw_set_calc.cycle += 1;
            gimbal_yaw_set_calc.angle -= (2*PI);
        }
        while(gimbal_yaw_set_calc.angle < 0){
            gimbal_yaw_set_calc.cycle -= 1;
            gimbal_yaw_set_calc.angle += (2*PI);
        }
        gimbal_yaw_set = (2*PI*gimbal_yaw_set_calc.cycle) + gimbal_yaw_set_calc.angle + init_yaw_pos;
        if (follow_chassis)
        {
            gimbal_yaw_set -= chassis_yaw_saved;
        }
        
    }else{
        chassis_yaw_saved = chassis_yaw;
    }
    if(state == Gimbal_Mode::FOLLOW_AUTOAIM)
    {
        gimbal_pitch_set = gimbal_pitch_now + pitch_autoaim * k_autoaim + x_offset;
        gimbal_yaw_set = gimbal_yaw_now - yaw_autoaim * k_autoaim + y_offset;
    }
    if(state == Gimbal_Mode::STEADY)
    {
        gimbal_pitch_set = pitch_saved;
        gimbal_yaw_set = yaw_saved;
    }else
    {
        yaw_saved = gimbal_yaw_set;
        pitch_saved = gimbal_pitch_set;
    }
    // ROS_INFO("%f",chassis_yaw);
    // std::cout<<range_diff<<" "<<gimbal_yaw_set<<" "<<gimbal_yaw_now<<std::endl;
    pitch_msg.data = (gimbal_pitch_set<pitch_lower_threshold?pitch_lower_threshold:gimbal_pitch_set);
    yaw_msg.data = gimbal_yaw_set;
    pitch_set_pub.publish(pitch_msg);
    yaw_set_pub.publish(yaw_msg);
}
