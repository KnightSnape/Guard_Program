#include"blackboard.hpp"

namespace decision_tree
{

void Blackboard::reset_flag()
{
    is_pitch_now_received = false;
    is_yaw_now_received = false;
    is_auto_aim_received = false;
    is_robot_hp_received = false;
    is_bullet_remain_received = false;
    is_game_status_received = false;
    is_game_result_received = false;
    is_robot_buff_received = false;
    is_robot_position_received = false;
    is_robot_status_received = false;
    is_dr16_receive_received = false;
    is_client_command_received = false;
    is_client_receive_received = false;
}

void Blackboard::init_params()
{
    std::string topic_path = ros::package::getPath("armor_decision") + "/config/topic.yaml";
    std::string graph_path = ros::package::getPath("armor_decision") + "/config/";
    std::string config_path = graph_path + "config.yaml";
    graph.initParam(graph_path);
    inference.init_param(graph_path);

    YAML::Node Config = YAML::LoadFile(config_path);
    command_control.max_control_time = Config["max_command_control_remain_time"].as<double>();
    shoot_control.max_control_time = Config["max_shoot_control_remain_time"].as<double>();
    standBy_control.max_control_time = Config["max_standby_control_remain_time"].as<double>();
    autoaim_control.max_control_time = Config["max_autoaim_control_remain_time"].as<double>();
    autoaim_state_control.max_control_time = Config["max_autoaim_state_control_remain_time"].as<double>();

    YAML::Node Topic = YAML::LoadFile(topic_path);
    pitch_now_topic = Topic["Subscribe"]["pitch_now_topic"].as<std::string>();
    yaw_now_topic = Topic["Subscribe"]["yaw_now_topic"].as<std::string>();
    auto_aim_sub_topic = Topic["Subscribe"]["auto_aim_sub_topic"].as<std::string>();
    robot_hp_sub_topic = Topic["Subscribe"]["robot_hp_sub_topic"].as<std::string>();
    bullet_remain_sub_topic = Topic["Subscribe"]["bullet_remain_sub_topic"].as<std::string>();
    game_state_sub_topic = Topic["Subscribe"]["game_state_sub_topic"].as<std::string>();
    game_result_sub_topic = Topic["Subscribe"]["game_result_sub_topic"].as<std::string>();
    robot_buff_sub_topic = Topic["Subscribe"]["robot_buff_sub_topic"].as<std::string>();
    robot_position_sub_topic = Topic["Subscribe"]["robot_position_sub_topic"].as<std::string>();
    robot_status_sub_topic = Topic["Subscribe"]["robot_status_sub_topic"].as<std::string>();
    client_command_sub_topic = Topic["Subscribe"]["client_command_sub_topic"].as<std::string>();
    dr16_receive_sub_topic = Topic["Subscribe"]["dr16_receive_sub_topic"].as<std::string>();
    client_receive_sub_topic = Topic["Subscribe"]["client_receive_sub_topic"].as<std::string>();
    vision_mode_switch_topic = Topic["Subscribe"]["vision_mode_switch_topic"].as<std::string>();
    odometry_topic = Topic["Subscribe"]["odometry_topic"].as<std::string>();
}

Blackboard::Blackboard():is_pitch_now_received(false),
                         is_yaw_now_received(false),
                         is_auto_aim_received(false),
                         is_robot_hp_received(false),
                         is_bullet_remain_received(false),
                         is_game_status_received(false),
                         is_game_result_received(false),
                         is_robot_buff_received(false),
                         is_robot_position_received(false),
                         is_robot_status_received(false),
                         is_client_command_received(false),
                         is_dr16_receive_received(false),
                         is_client_receive_received(false),
                         is_odometry_received(false)
{
    init_params();
    navigation_target_id = 0;

    chassis_mode = Chassis_Mode::INITIAL;
    gimbal_mode = Gimbal_Mode::HIGH_SPEED;
    last_gimbal_mode = Gimbal_Mode::NO_FORCE;
    command_mode = CMD_Command::INITIAL;
    autoaim_mode = AutoAim_Mode::INITIAL;
    
    //注意初始化在启动区，越偏离放置位越容易出事(TODO: 使用uwb初始化它，当开机启动后收到的第一个点就是它放置的点)
    sentry_map_point[0] = 170;
    sentry_map_point[1] = 225;   

    pos_manager.set_pos_x_zero(sentry_map_point[0]);
    pos_manager.set_pos_y_zero(sentry_map_point[1]);

    now_id = 0;
    
    average_time = 0.01;

    pitch_now_sub = nh_.subscribe(pitch_now_topic,10,&Blackboard::Pitch_Now_Callback,this);
    yaw_now_sub = nh_.subscribe(yaw_now_topic,10,&Blackboard::Yaw_Now_Callback,this);
    auto_aim_sub = nh_.subscribe(auto_aim_sub_topic,10,&Blackboard::Auto_Aim_Callback,this);
    robot_hp_sub = nh_.subscribe(robot_hp_sub_topic,10,&Blackboard::Robot_Hp_Callback,this);
    bullet_remain_sub = nh_.subscribe(bullet_remain_sub_topic,10,&Blackboard::Bullet_Remain_Callback,this);
    game_status_sub = nh_.subscribe(game_state_sub_topic,10,&Blackboard::Game_State_Callback,this);
    game_result_sub = nh_.subscribe(game_result_sub_topic,10,&Blackboard::Game_Result_Callback,this);
    robot_buff_sub = nh_.subscribe(robot_buff_sub_topic,10,&Blackboard::Robot_Buff_Callback,this);
    robot_position_sub = nh_.subscribe(robot_position_sub_topic,10,&Blackboard::Robot_Position_Callback,this);
    robot_status_sub = nh_.subscribe(robot_status_sub_topic,10,&Blackboard::Robot_Status_Callback,this);
    client_command_sub = nh_.subscribe(client_command_sub_topic,10,&Blackboard::Client_Command_Callback,this);
    dr16_receive_sub = nh_.subscribe(dr16_receive_sub_topic,10,&Blackboard::DR16_Receive_Callback,this);
    client_receive_sub = nh_.subscribe(client_receive_sub_topic,10,&Blackboard::Client_Receive_Callback,this);
    odometry_sub = nh_.subscribe(odometry_topic,10,&Blackboard::Odometry_Callback,this);
    //vision_mode_switch_pub = nh_.advertiseService(vision_mode_switch_topic,10);
}

void Blackboard::Pitch_Now_Callback(const gary_msgs::DualLoopPIDWithFilter::ConstPtr msg)
{
    pitch_now_mutex.lock();
    pitch_now_msg = *msg;
    is_pitch_now_received = true;
    pitch_now_mutex.unlock();  
}

void Blackboard::Yaw_Now_Callback(const gary_msgs::DualLoopPIDWithFilter::ConstPtr msg)
{
    yaw_now_mutex.lock();
    yaw_now_msg = *msg;
    is_yaw_now_received = true;
    yaw_now_mutex.unlock();
}

void Blackboard::Auto_Aim_Callback(const gary_msgs::AutoAIM::ConstPtr msg)
{
    auto_aim_mutex.lock();
    auto_aim_msg = *msg;
    is_auto_aim_received = true;
    auto_aim_mutex.unlock();
}

void Blackboard::Robot_Hp_Callback(const gary_msgs::RobotHP::ConstPtr msg)
{
    robot_hp_mutex.lock();
    robot_hp_msg = *msg;
    is_robot_hp_received = true;
    robot_hp_mutex.unlock();
}

void Blackboard::Bullet_Remain_Callback(const gary_msgs::BulletRemaining::ConstPtr msg)
{
    bullet_remain_mutex.lock();
    bullet_remain_msg = *msg;
    is_bullet_remain_received = true;
    bullet_remain_mutex.unlock();
}

void Blackboard::Game_State_Callback(const gary_msgs::GameStatus::ConstPtr msg)
{
    game_status_mutex.lock();
    game_status_msg = *msg;
    is_game_result_received = true;
    game_status_mutex.unlock();
}

void Blackboard::Game_Result_Callback(const gary_msgs::GameResult::ConstPtr msg)
{
    game_result_mutex.lock();
    game_result_msg = *msg;
    is_game_result_received = true;
    game_result_mutex.unlock();
}

void Blackboard::Robot_Buff_Callback(const gary_msgs::RobotBuff::ConstPtr msg)
{
    robot_buff_mutex.lock();
    robot_buff_msg = *msg;
    is_robot_buff_received = true;
    robot_buff_mutex.unlock();
}

void Blackboard::Robot_Position_Callback(const gary_msgs::RobotPosition::ConstPtr msg)
{
    robot_position_mutex.lock();
    robot_position_msg = *msg;
    is_robot_buff_received = true;
    robot_position_mutex.unlock();
}

void Blackboard::Robot_Status_Callback(const gary_msgs::RobotStatus::ConstPtr msg)
{
    robot_status_mutex.lock();
    robot_status_msg = *msg;
    is_robot_status_received = true;
    robot_status_mutex.unlock();
}

void Blackboard::Client_Command_Callback(const gary_msgs::ClientCommand::ConstPtr msg)
{
    client_command_mutex.lock();
    client_command_msg = *msg;
    is_client_command_received = true;
    Client_Command_State_Transform();
    client_command_mutex.unlock();
}

void Blackboard::DR16_Receive_Callback(const gary_msgs::DR16Receiver::ConstPtr msg)
{
    dr16_receive_mutex.lock();
    dr16_receive_msg = *msg;
    is_dr16_receive_received = true;
    dr16_receive_mutex.unlock();
}

void Blackboard::Client_Receive_Callback(const gary_msgs::ClientReceive::ConstPtr msg)
{
    client_receive_mutex.lock();
    client_receive_msg = *msg;
    is_client_receive_received = true;
    client_receive_mutex.unlock();
}

void Blackboard::Odometry_Callback(const nav_msgs::Odometry::ConstPtr msg)
{
    odometry_mutex.lock();
    odometry_msg = *msg;
    is_odometry_received = true;
    pos_manager.set_guard_world_pos(odometry_msg);
    pos_manager.set_guard_rotate_q(odometry_msg);
    pos_manager.setRPY();
    sentry_map_point = pos_manager.robot_to_map(pos_manager.get_guard_world_pos());
    if(robot_status_msg.robot_id == (uint8_t)107)
    sentry_map_point = pos_manager.inverse_point(sentry_map_point);
    odometry_mutex.unlock();
}

void Blackboard::Client_Command_State_Transform()
{
    if((char)client_command_msg.keyboard_key_pressed == 'Q')
    {
        ROS_INFO("Receive to cmd command: MOVE_TARGET");
        command_mode = CMD_Command::MOVE_TARGET;
    }
    else if((char)client_command_msg.keyboard_key_pressed == 'R')
    {
        ROS_INFO("Receive to cmd command: REPOS");
        command_mode = CMD_Command::REPOS;
    }
    else if((char)client_command_msg.keyboard_key_pressed == 'Z')
    {
        ROS_INFO("Receive to cmd command: STOP_MOVING");
        command_mode = CMD_Command::STOP_MOVING;
    }
    else if((char)client_command_msg.keyboard_key_pressed == 'X')
    {
        ROS_INFO("Receive to cmd command: STOP_SHOOT");
        command_mode = CMD_Command::STOP_SHOOT;
    }
    else if((char)client_command_msg.keyboard_key_pressed == 'E')
    {
        ROS_INFO("Receive to cmd command: MOVE_UP");
        command_mode = CMD_Command::MOVE_UP;
    }
    else if((char)client_command_msg.keyboard_key_pressed == 'D')
    {
        ROS_INFO("Receive to cmd command: MOVE_DOWN");
        command_mode = CMD_Command::MOVE_DOWN;
    }
    else if((char)client_command_msg.keyboard_key_pressed == 'S')
    {
        ROS_INFO("Receive to cmd command: MOVE_LEFT");
        command_mode = CMD_Command::MOVE_LEFT;
    }
    else if((char)client_command_msg.keyboard_key_pressed == 'F')
    {
        ROS_INFO("Receive to cmd command: MOVE_RIGHT");
        command_mode = CMD_Command::MOVE_RIGHT;
    }
    else if((char)client_command_msg.keyboard_key_pressed == 'Y')
    {
        ROS_INFO("Receive to cmd command: MOVE_RIGHT_UP");
        command_mode = CMD_Command::MOVE_RIGHT_UP;
    }
    else if((char)client_command_msg.keyboard_key_pressed == 'H')
    {
        ROS_INFO("Receive to cmd command: MOVE_RIGHT_DOWN");
        command_mode = CMD_Command::MOVE_RIGHT_DOWN;
    }
    else if((char)client_command_msg.keyboard_key_pressed == 'G')
    {
        ROS_INFO("Receive to cmd command: MOVE_LEFT_UP");
        command_mode = CMD_Command::MOVE_LEFT_UP;
    }
    else if((char)client_command_msg.keyboard_key_pressed == 'J')
    {
        ROS_INFO("Receive to cmd command: MOVE_LEFT_DOWN");
        command_mode = CMD_Command::MOVE_LEFT_DOWN;
    }
    else if((char)client_command_msg.keyboard_key_pressed == 'V')
    {
        ROS_INFO("Receive to cmd command: TARGET_TO_HOME");
        command_mode = CMD_Command::TARGET_TO_HOME;
    }
    else if((char)client_command_msg.keyboard_key_pressed == 'N')
    {
        ROS_INFO("Receive to cmd command: TARGET_TO_PATRAL_AREA");
        command_mode = CMD_Command::TARGET_TO_PATRAL_AREA;
    }
    else if((char)client_command_msg.keyboard_key_pressed == 'K')
    {
        ROS_INFO("Receive to cmd command: TRANSFORM_TO_HIGH_SPEED");
        command_mode = CMD_Command::TRANSFORM_TO_HIGH_SPEED;
    }
    else if((char)client_command_msg.keyboard_key_pressed == 'L')
    {
        ROS_INFO("Receive to cmd command: TARGET_TO_MY_STEP");
        command_mode = CMD_Command::TRANSFORM_TO_SCANNING;
    }
    else if((char)client_command_msg.keyboard_key_pressed == 'U')
    {
        ROS_INFO("Receive to cmd command: TARGET_TO_ENEMY_OUTPOST");
        command_mode = CMD_Command::TARGET_TO_ENEMY_OUTPOST;
    }
    else if((char)client_command_msg.keyboard_key_pressed == 'O')
    {
        ROS_INFO("Receive to cmd command: TARGET_TO_ENEMY_PATRAL");
        command_mode = CMD_Command::TARGET_TO_ENEMY_PATRAL;
    }
    else if((char)client_command_msg.keyboard_key_pressed == 'C')
    {
        ROS_INFO("Receive to cmd command: START_ROTATE");
        command_mode = CMD_Command::START_ROTATE;
    }
    else if((char)client_command_msg.keyboard_key_pressed == 'T')
    {
        ROS_INFO("Receive to cmd command: STOP_ROTATE");
        command_mode = CMD_Command::STOP_ROTATE;
    }
    else if((char)client_command_msg.keyboard_key_pressed == (char)0)
    {
        ROS_INFO("Receive to cmd command: LOOKAT");
        command_mode = CMD_Command::LOOKAT;
    }
    

}


}