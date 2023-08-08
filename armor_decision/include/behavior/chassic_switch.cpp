#include"chassic_switch.hpp"

namespace decision_tree
{

Chassis_Switch::Chassis_Switch(std::string name,
                               int level,
                               const Blackboard::Ptr &blackboard,
                               const Chassis_executor::Ptr &chassis_executor,
                               const Gimbal_executor::Ptr &gimbal_executor,
                               const Log_executor::Ptr &log_executor,
                               const AutoAim_executor::Ptr &autoaim_executor):
                               ActionNode::ActionNode(name,level,blackboard,chassis_executor,gimbal_executor,log_executor,autoaim_executor){}

BehaviorState Chassis_Switch::Update()
{
    cv::Point sentry_point = cv::Point(blackboard_ptr_->sentry_map_point.x(),blackboard_ptr_->sentry_map_point.y());
    if(blackboard_ptr_->dr16_receive_msg.sw_right != blackboard_ptr_->dr16_receive_msg.SW_UP)
    {
        return BehaviorState::SUCCESS;
    }
    //已经接收到了云台手指令
    if(blackboard_ptr_->is_client_command_received)
    {
        //查看是否为控制底盘指令，直接刷新成云台手控制模式
        if(checkchassic())
        {
            //如果是正在导航模式，即刻停止
            if(blackboard_ptr_->chassis_mode == Chassis_Mode::NAVIGATING)
            {
                ROS_DEBUG("Receive CMD,stop navigating");
                chassis_exe_ptr_->pub_stop_signal();
            }
            blackboard_ptr_->chassis_mode = Chassis_Mode::LISTEN_CAPTAIN;
        }
    }
    if(blackboard_ptr_->chassis_mode == Chassis_Mode::INITIAL)
        return BehaviorState::SUCCESS;
    //正在导航状态 
    if(blackboard_ptr_->chassis_mode == Chassis_Mode::NAVIGATING)
    {
        ROS_DEBUG("Is navigating");
        //导航如果已经到达就可以切成STANDBY模式，使其停留或者小陀螺一段时间，如果到达一个局部目标，使用局部模式
        int target_state;
        //更新一轮
        blackboard_ptr_->graph.updatePoint(sentry_point,blackboard_ptr_->now_id,blackboard_ptr_->navigation_target_id,
                                            target_state,blackboard_ptr_->next_target_id);
        ROS_WARN("You are in id: %d,on point:(%d,%d)",blackboard_ptr_->now_id,sentry_point.x,sentry_point.y);
        ROS_WARN("Your target is: %d,and the next target is: %d",blackboard_ptr_->navigation_target_id,blackboard_ptr_->next_target_id);
        ROS_WARN("Target State is: %d",target_state);
        //如果已经到达目标，使用STANDBY模式停留
        if(target_state == 1)
        {
            ROS_INFO("On Target,You are in: %d",blackboard_ptr_->now_id);
            chassis_exe_ptr_->pub_stop_signal();
            blackboard_ptr_->standBy_control.is_standby = true;
            blackboard_ptr_->standBy_control.remain_control_time = blackboard_ptr_->standBy_control.max_control_time;
            blackboard_ptr_->chassis_mode = Chassis_Mode::STANDBY;
        }
        else if(target_state == 2)
        {
            ROS_DEBUG("GOTO Next Target");
            cv::Point target_point = blackboard_ptr_->graph.get_node_point(blackboard_ptr_->next_target_id);
            Eigen::Vector2i target_point_eigen{target_point.x,target_point.y};
            if(blackboard_ptr_->robot_status_msg.robot_id == (uint8_t)107)
            target_point_eigen = blackboard_ptr_->pos_manager.inverse_point(target_point_eigen);
            Eigen::Vector3d target_pos_world = blackboard_ptr_->pos_manager.map_to_world(target_point_eigen);
            geometry_msgs::PointStamped target_pos_msg = blackboard_ptr_->pos_manager.transfer_to_msg(target_pos_world);
            chassis_exe_ptr_->pub_nav_point(target_pos_msg);
        }
        //底盘状态完成
        return BehaviorState::SUCCESS;
    }
    //跟随云台状态
    if(blackboard_ptr_->chassis_mode == Chassis_Mode::FOLLOW_GIMBAL)
    {
        //TODO(Knight):Add this method
        return BehaviorState::SUCCESS;
    }
    //当前已经作出云台手指令，在完成相应动作后直接
    if(blackboard_ptr_->chassis_mode == Chassis_Mode::LISTEN_CAPTAIN)
    {
        //如果是手控模式转换成手控
        if((blackboard_ptr_->command_mode == CMD_Command::MOVE_UP) || 
            (blackboard_ptr_->command_mode == CMD_Command::MOVE_DOWN) || 
            (blackboard_ptr_->command_mode == CMD_Command::MOVE_LEFT) || 
            (blackboard_ptr_->command_mode == CMD_Command::MOVE_RIGHT) || 
            (blackboard_ptr_->command_mode == CMD_Command::MOVE_UP_ABS) ||
            (blackboard_ptr_->command_mode == CMD_Command::MOVE_DOWN_ABS) || 
            (blackboard_ptr_->command_mode == CMD_Command::MOVE_LEFT_ABS) || 
            (blackboard_ptr_->command_mode == CMD_Command::MOVE_RIGHT_ABS) || 
            (blackboard_ptr_->command_mode == CMD_Command::STOP_MOVING) || 
            (blackboard_ptr_->command_mode == CMD_Command::MOVE_RIGHT_DOWN) || 
            (blackboard_ptr_->command_mode == CMD_Command::MOVE_RIGHT_UP) || 
            (blackboard_ptr_->command_mode == CMD_Command::MOVE_LEFT_UP) ||
            (blackboard_ptr_->command_mode == CMD_Command::MOVE_LEFT_DOWN) ||
            (blackboard_ptr_->command_mode == CMD_Command::LOOKAT)) 
            {
                ROS_DEBUG("Switch to captain control mode");
                blackboard_ptr_->chassis_mode = Chassis_Mode::CAPTAIN_CONTROL;
                blackboard_ptr_->command_control.is_control = true;
                blackboard_ptr_->command_control.remain_control_time = blackboard_ptr_->command_control.max_control_time;
            }
        if(blackboard_ptr_->command_mode == CMD_Command::MOVE_TARGET)
        {
            Eigen::Vector3d target_eigen{blackboard_ptr_->client_command_msg.target_position_x,
                                         blackboard_ptr_->client_command_msg.target_position_y,
                                         blackboard_ptr_->client_command_msg.target_position_z};
            Eigen::Vector2i target_2d = blackboard_ptr_->pos_manager.world_to_map(target_eigen);
            if(blackboard_ptr_->robot_status_msg.robot_id == (uint8_t)107)
            target_2d = blackboard_ptr_->pos_manager.inverse_point(target_2d);
            cv::Point target_point = cv::Point(target_2d.x(),target_2d.y());
            int id = blackboard_ptr_->graph.CheckInPoint(target_point);
            if(id == -1)
            {
                ROS_WARN("Wrong Place");
                blackboard_ptr_->standBy_control.is_standby = true;
                blackboard_ptr_->standBy_control.remain_control_time = blackboard_ptr_->standBy_control.max_control_time;
                blackboard_ptr_->chassis_mode = Chassis_Mode::STANDBY;
                return BehaviorState::SUCCESS;
            }
            blackboard_ptr_->navigation_target_id = id;
            //如果在其他位置，进入navigation模式，并发送导航所对应的目标
            //ROS_WARN("the point is (%d,%d)",sentry_point.x,sentry_point.y);
            blackboard_ptr_->now_id = blackboard_ptr_->graph.CheckInPoint(sentry_point);
            if(blackboard_ptr_->now_id == -1)
            {
                blackboard_ptr_->next_target_id = blackboard_ptr_->graph.findLatestpoint(sentry_point);
            }
            else
            {
                blackboard_ptr_->next_target_id = blackboard_ptr_->graph.get_first_point(blackboard_ptr_->now_id,blackboard_ptr_->navigation_target_id);
            }
            ROS_WARN("now id is: %d,the target id is:%d,the next target id is:%d",blackboard_ptr_->now_id,id,blackboard_ptr_->next_target_id);
            cv::Point target_true_point = blackboard_ptr_->graph.get_node_point(blackboard_ptr_->next_target_id);
            Eigen::Vector2i target_point_eigen{target_true_point.x,target_true_point.y};
            if(blackboard_ptr_->robot_status_msg.robot_id == (uint8_t)107)
            target_point_eigen = blackboard_ptr_->pos_manager.inverse_point(target_point_eigen);
            Eigen::Vector3d target_pos_world = blackboard_ptr_->pos_manager.map_to_world(target_point_eigen);
            ROS_DEBUG("we are in:(%d,%d)",sentry_point.x,sentry_point.y);   
            ROS_DEBUG("the target is in:(%d,%d)",target_point_eigen.x(),target_point_eigen.y());
            ROS_DEBUG("the target 3d point is:(%f,%f,%f)",target_pos_world.x(),target_pos_world.y(),target_pos_world.z());
            geometry_msgs::PointStamped target_pos_msg = blackboard_ptr_->pos_manager.transfer_to_msg(target_pos_world);
            chassis_exe_ptr_->pub_nav_point(target_pos_msg);
            ROS_DEBUG("navigation start");
            blackboard_ptr_->chassis_mode = Chassis_Mode::NAVIGATING;
            return BehaviorState::SUCCESS;
        }
        if(checktarget())
        {
            cv::Point target_point_2d;
            if(blackboard_ptr_->command_mode == CMD_Command::TARGET_TO_ENEMY_OUTPOST)
            {
                target_point_2d = cv::Point(460,88);
            }
            if(blackboard_ptr_->command_mode == CMD_Command::TARGET_TO_ENEMY_PATRAL)
            {
                target_point_2d = cv::Point(619,228);
            }
            if(blackboard_ptr_->command_mode == CMD_Command::TARGET_TO_HOME)
            {
                target_point_2d = cv::Point(170,225);
            }
            if(blackboard_ptr_->command_mode == CMD_Command::TARGET_TO_MY_OUTPOST)
            {
                target_point_2d = cv::Point(378,367);
            }
            if(blackboard_ptr_->command_mode == CMD_Command::TARGET_TO_MY_STEP)
            {
                target_point_2d = cv::Point(381,18);
            }
            if(blackboard_ptr_->command_mode == CMD_Command::TARGET_TO_PATRAL_AREA)
            {
                target_point_2d = cv::Point(105,16);
            }
            int id = blackboard_ptr_->graph.CheckInPoint(target_point_2d);
            if(id == -1)
            {
                ROS_ERROR("Initial Wrong");
                blackboard_ptr_->standBy_control.is_standby = true;
                blackboard_ptr_->standBy_control.remain_control_time = blackboard_ptr_->standBy_control.max_control_time;
                blackboard_ptr_->chassis_mode = Chassis_Mode::STANDBY;
                return BehaviorState::SUCCESS;
            }
            blackboard_ptr_->navigation_target_id = id;
            //如果在其他位置，进入navigation模式，并发送导航所对应的目标
            //ROS_WARN("the point is (%d,%d)",sentry_point.x,sentry_point.y);
            blackboard_ptr_->now_id = blackboard_ptr_->graph.CheckInPoint(sentry_point);
            if(blackboard_ptr_->now_id == -1)
            {
                blackboard_ptr_->next_target_id = blackboard_ptr_->graph.findLatestpoint(sentry_point);
            }
            else
            {
                blackboard_ptr_->next_target_id = blackboard_ptr_->graph.get_first_point(blackboard_ptr_->now_id,blackboard_ptr_->navigation_target_id);
            }
            ROS_WARN("now id is: %d,the target id is:%d,the next target id is:%d",blackboard_ptr_->now_id,id,blackboard_ptr_->next_target_id);
            cv::Point target_true_point = blackboard_ptr_->graph.get_node_point(blackboard_ptr_->next_target_id);
            Eigen::Vector2i target_point_eigen{target_true_point.x,target_true_point.y};
            if(blackboard_ptr_->robot_status_msg.robot_id == (uint8_t)107)
            target_point_eigen = blackboard_ptr_->pos_manager.inverse_point(target_point_eigen);
            Eigen::Vector3d target_pos_world = blackboard_ptr_->pos_manager.map_to_world(target_point_eigen);
            ROS_DEBUG("we are in:(%d,%d)",sentry_point.x,sentry_point.y);   
            ROS_DEBUG("the target is in:(%d,%d)",target_point_eigen.x(),target_point_eigen.y());
            ROS_DEBUG("the target 3d point is:(%f,%f,%f)",target_pos_world.x(),target_pos_world.y(),target_pos_world.z());
            geometry_msgs::PointStamped target_pos_msg = blackboard_ptr_->pos_manager.transfer_to_msg(target_pos_world);
            chassis_exe_ptr_->pub_nav_point(target_pos_msg);
            ROS_DEBUG("navigation start");
            blackboard_ptr_->chassis_mode = Chassis_Mode::NAVIGATING;
            return BehaviorState::SUCCESS;
        }
        if(blackboard_ptr_->command_mode == CMD_Command::REPOS)
        {
            //TODO(Knight):Add method
        }
        if(blackboard_ptr_->command_mode == CMD_Command::START_ROTATE)
        {
            ROS_INFO("switch to rotate");
            chassis_exe_ptr_->pub_stop_signal();
            geometry_msgs::Twist twist;
            twist.linear.x = 0.0;
            twist.linear.y = 0.0;
            twist.linear.z = 0.0;
            twist.angular.x = 0.0;
            twist.angular.y = 0.0;
            twist.angular.z = 420.0 / 180 * M_PI;
            chassis_exe_ptr_->pub_twist(twist);
            blackboard_ptr_->chassis_mode = Chassis_Mode::ROTATE;
            blackboard_ptr_->gimbal_mode = Gimbal_Mode::SCANNING;
            return BehaviorState::SUCCESS;
        }
        if(blackboard_ptr_->command_mode == CMD_Command::STOP_ROTATE)
        {
            chassis_exe_ptr_->pub_stop_signal();
            geometry_msgs::Twist twist;
            twist.linear.x = 0.0;
            twist.linear.y = 0.0;
            twist.linear.z = 0.0;
            twist.angular.x = 0.0;
            twist.angular.y = 0.0;
            twist.angular.z = 0.0;
            chassis_exe_ptr_->pub_twist(twist);
            blackboard_ptr_->chassis_mode = Chassis_Mode::STANDBY;
            return BehaviorState::SUCCESS;
        }

    }
    //云台手控制模式
    if(blackboard_ptr_->chassis_mode == Chassis_Mode::CAPTAIN_CONTROL)
    {
        float twist_yaw_angle = 0;
        //直接控制方向移动
        if(blackboard_ptr_->command_mode == CMD_Command::MOVE_UP)
        {
            ROS_INFO("start control up");
            if(blackboard_ptr_->robot_status_msg.robot_id < (uint8_t)20)
            {
                blackboard_ptr_->pos_manager.speed_process(blackboard_ptr_->command_control.twist_x,blackboard_ptr_->command_control.twist_y,
                                                       4,twist_yaw_angle);
            }
            else
            {
                blackboard_ptr_->pos_manager.speed_process(blackboard_ptr_->command_control.twist_x,blackboard_ptr_->command_control.twist_y,
                                                       2,twist_yaw_angle);   
            }
        }
        else if(blackboard_ptr_->command_mode == CMD_Command::MOVE_DOWN)
        {
            ROS_DEBUG("start control down");
            if(blackboard_ptr_->robot_status_msg.robot_id < (uint8_t)20)
            {
                blackboard_ptr_->pos_manager.speed_process(blackboard_ptr_->command_control.twist_x,blackboard_ptr_->command_control.twist_y,
                                                        2,twist_yaw_angle);
            }
            else
            {
                blackboard_ptr_->pos_manager.speed_process(blackboard_ptr_->command_control.twist_x,blackboard_ptr_->command_control.twist_y,
                                                        4,twist_yaw_angle);
            }

        }
        else if(blackboard_ptr_->command_mode == CMD_Command::MOVE_LEFT)
        {
            ROS_DEBUG("start control left");
            if(blackboard_ptr_->robot_status_msg.robot_id < (uint8_t)20)
            {
                blackboard_ptr_->pos_manager.speed_process(blackboard_ptr_->command_control.twist_x,blackboard_ptr_->command_control.twist_y,
                                                        3,twist_yaw_angle);
            }
            else
            {
                blackboard_ptr_->pos_manager.speed_process(blackboard_ptr_->command_control.twist_x,blackboard_ptr_->command_control.twist_y,
                                                        1,twist_yaw_angle);
            }
        }
        else if(blackboard_ptr_->command_mode == CMD_Command::MOVE_RIGHT)
        {
            ROS_DEBUG("start control right");
            if(blackboard_ptr_->robot_status_msg.robot_id < (uint8_t)20)
            {
                blackboard_ptr_->pos_manager.speed_process(blackboard_ptr_->command_control.twist_x,blackboard_ptr_->command_control.twist_y,
                                                        1,twist_yaw_angle);
            }
            else
            {
                blackboard_ptr_->pos_manager.speed_process(blackboard_ptr_->command_control.twist_x,blackboard_ptr_->command_control.twist_y,
                                                        3,twist_yaw_angle);              
            }

        }
        else if(blackboard_ptr_->command_mode == CMD_Command::MOVE_RIGHT_UP)
        {
            if(blackboard_ptr_->robot_status_msg.robot_id < (uint8_t)20)
            {
                blackboard_ptr_->pos_manager.speed_process(blackboard_ptr_->command_control.twist_x,blackboard_ptr_->command_control.twist_y,
                                                        5,twist_yaw_angle);
            }
            else
            {
                blackboard_ptr_->pos_manager.speed_process(blackboard_ptr_->command_control.twist_x,blackboard_ptr_->command_control.twist_y,
                                                        8,twist_yaw_angle);              
            } 
        }
        else if(blackboard_ptr_->command_mode == CMD_Command::MOVE_RIGHT_DOWN)
        {
            if(blackboard_ptr_->robot_status_msg.robot_id < (uint8_t)20)
            {
                blackboard_ptr_->pos_manager.speed_process(blackboard_ptr_->command_control.twist_x,blackboard_ptr_->command_control.twist_y,
                                                        7,twist_yaw_angle);
            }
            else
            {
                blackboard_ptr_->pos_manager.speed_process(blackboard_ptr_->command_control.twist_x,blackboard_ptr_->command_control.twist_y,
                                                        6,twist_yaw_angle);              
            }  
        }
        else if(blackboard_ptr_->command_mode == CMD_Command::MOVE_LEFT_UP)
        {
            if(blackboard_ptr_->robot_status_msg.robot_id < (uint8_t)20)
            {
                blackboard_ptr_->pos_manager.speed_process(blackboard_ptr_->command_control.twist_x,blackboard_ptr_->command_control.twist_y,
                                                        6,twist_yaw_angle);
            }
            else
            {
                blackboard_ptr_->pos_manager.speed_process(blackboard_ptr_->command_control.twist_x,blackboard_ptr_->command_control.twist_y,
                                                        7,twist_yaw_angle);              
            } 
        }
        else if(blackboard_ptr_->command_mode == CMD_Command::MOVE_RIGHT_DOWN)
        {
            if(blackboard_ptr_->robot_status_msg.robot_id < (uint8_t)20)
            {
                blackboard_ptr_->pos_manager.speed_process(blackboard_ptr_->command_control.twist_x,blackboard_ptr_->command_control.twist_y,
                                                        8,twist_yaw_angle);
            }
            else
            {
                blackboard_ptr_->pos_manager.speed_process(blackboard_ptr_->command_control.twist_x,blackboard_ptr_->command_control.twist_y,
                                                        5,twist_yaw_angle);              
            }
        }
        else if(blackboard_ptr_->command_mode == CMD_Command::MOVE_UP_ABS)
        {
            ROS_DEBUG("start control up abs");
            blackboard_ptr_->command_control.twist_x = 0.1;
            blackboard_ptr_->command_control.twist_y = 0;
        }
        else if(blackboard_ptr_->command_mode == CMD_Command::MOVE_DOWN_ABS)
        {
            ROS_DEBUG("start control down abs");
            blackboard_ptr_->command_control.twist_x = -0.1;
            blackboard_ptr_->command_control.twist_y = 0;
        }
        else if(blackboard_ptr_->command_mode == CMD_Command::MOVE_LEFT_ABS)
        {
            ROS_DEBUG("start control left abs");
            blackboard_ptr_->command_control.twist_x = 0;
            blackboard_ptr_->command_control.twist_y = 0.1;
        }
        else if(blackboard_ptr_->command_mode == CMD_Command::MOVE_RIGHT_ABS)
        {
            ROS_DEBUG("start control right abs");
            blackboard_ptr_->command_control.twist_x = 0;
            blackboard_ptr_->command_control.twist_y = -0.1;
        }
        else if(blackboard_ptr_->command_mode == CMD_Command::STOP_MOVING)
        {
            blackboard_ptr_->command_control.twist_x = 0;
            blackboard_ptr_->command_control.twist_y = 0;
        }
        geometry_msgs::Twist twist;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = twist_yaw_angle;
        twist.linear.x = blackboard_ptr_->command_control.twist_x;
        twist.linear.y = blackboard_ptr_->command_control.twist_y;
        twist.linear.z = 0.0;
        chassis_exe_ptr_->pub_twist(twist);
        //剩余时间控制
        if(twist_yaw_angle == 0)
        blackboard_ptr_->command_control.remain_control_time = blackboard_ptr_->command_control.remain_control_time - blackboard_ptr_->average_time;
        if(blackboard_ptr_->command_control.remain_control_time <= 0)
        {
            ROS_DEBUG("Captain Control Done");
            blackboard_ptr_->command_control.remain_control_time = 0.0;
            blackboard_ptr_->command_control.is_control = false;
            //立即进入自由位置判断
            blackboard_ptr_->chassis_mode = Chassis_Mode::FREE_CAPTAIN;
        }
        return BehaviorState::SUCCESS;
    }
    
    //原地模式
    if(blackboard_ptr_->chassis_mode == Chassis_Mode::STANDBY)
    {
        blackboard_ptr_->standBy_control.remain_control_time = blackboard_ptr_->standBy_control.remain_control_time - blackboard_ptr_->average_time;
        if(blackboard_ptr_->standBy_control.remain_control_time <= 0)
        {
            ROS_DEBUG("StandBy over");
            blackboard_ptr_->standBy_control.remain_control_time = 0.0;
            blackboard_ptr_->standBy_control.is_standby = false;
            //再次进入自由决策模式
            blackboard_ptr_->chassis_mode = Chassis_Mode::FREE_CAPTAIN;
        }
        return BehaviorState::SUCCESS;
    }

    if(blackboard_ptr_->chassis_mode == Chassis_Mode::ROTATE)
    {
        geometry_msgs::Twist twist;
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 420.0 / 180 * M_PI;
        chassis_exe_ptr_->pub_twist(twist);
        return BehaviorState::SUCCESS;
    }

    if(blackboard_ptr_->chassis_mode == Chassis_Mode::FREE_CAPTAIN)
    {
        blackboard_ptr_->inference.main_process(blackboard_ptr_->navigation_target_id,blackboard_ptr_->gimbal_mode,blackboard_ptr_->autoaim_mode);
        //如果你的decision目标就是原地，可以保持不动，使用STANDBY模式
        if(blackboard_ptr_->now_id == blackboard_ptr_->navigation_target_id)
        {
            ROS_DEBUG("don't go anywhere");
            blackboard_ptr_->standBy_control.is_standby = true;
            blackboard_ptr_->standBy_control.remain_control_time = blackboard_ptr_->standBy_control.max_control_time;
            blackboard_ptr_->chassis_mode = Chassis_Mode::STANDBY;
        }
        else
        {
            //如果在其他位置，进入navigation模式，并发送导航所对应的目标
            blackboard_ptr_->now_id = blackboard_ptr_->graph.CheckInPoint(sentry_point);
            if(blackboard_ptr_->now_id == -1)
            {
                blackboard_ptr_->next_target_id = blackboard_ptr_->graph.findLatestpoint(sentry_point);
            }
            else
            {
                blackboard_ptr_->next_target_id = blackboard_ptr_->graph.get_first_point(blackboard_ptr_->now_id,blackboard_ptr_->navigation_target_id);
            }
            cv::Point target_point = blackboard_ptr_->graph.get_node_point(blackboard_ptr_->next_target_id);
            Eigen::Vector2i target_point_eigen{target_point.x,target_point.y};
            if(blackboard_ptr_->robot_status_msg.robot_id == (uint8_t)107)
            target_point_eigen = blackboard_ptr_->pos_manager.inverse_point(target_point_eigen);
            Eigen::Vector3d target_pos_world = blackboard_ptr_->pos_manager.map_to_world(target_point_eigen);
            ROS_DEBUG("we are in:(%d,%d)",sentry_point.x,sentry_point.y);   
            ROS_DEBUG("the target is in:(%d,%d)",target_point_eigen.x(),target_point_eigen.y());
            ROS_DEBUG("the target 3d point is:(%f,%f,%f)",target_pos_world.x(),target_pos_world.y(),target_pos_world.z());
            geometry_msgs::PointStamped target_pos_msg = blackboard_ptr_->pos_manager.transfer_to_msg(target_pos_world);
            chassis_exe_ptr_->pub_nav_point(target_pos_msg);
            ROS_DEBUG("navigation start");
            blackboard_ptr_->chassis_mode = Chassis_Mode::NAVIGATING;
        }
        return BehaviorState::SUCCESS;
    }
}

bool Chassis_Switch::checkchassic()
{
    bool check = (blackboard_ptr_->command_mode == CMD_Command::MOVE_DOWN) ||
                    (blackboard_ptr_->command_mode == CMD_Command::MOVE_RIGHT) ||
                    (blackboard_ptr_->command_mode == CMD_Command::MOVE_LEFT) ||
                    (blackboard_ptr_->command_mode == CMD_Command::MOVE_UP) ||
                    (blackboard_ptr_->command_mode == CMD_Command::MOVE_TARGET) ||
                    (blackboard_ptr_->command_mode == CMD_Command::STOP_MOVING) || 
                    (blackboard_ptr_->command_mode == CMD_Command::REPOS) ||
                    (blackboard_ptr_->command_mode == CMD_Command::MOVE_UP_ABS) ||
                    (blackboard_ptr_->command_mode == CMD_Command::MOVE_DOWN_ABS) || 
                    (blackboard_ptr_->command_mode == CMD_Command::MOVE_LEFT_ABS) || 
                    (blackboard_ptr_->command_mode == CMD_Command::MOVE_RIGHT_ABS) || 
                    (blackboard_ptr_->command_mode == CMD_Command::START_ROTATE) || 
                    (blackboard_ptr_->command_mode == CMD_Command::STOP_ROTATE)|| 
                    (blackboard_ptr_->command_mode == CMD_Command::MOVE_RIGHT_DOWN) || 
                    (blackboard_ptr_->command_mode == CMD_Command::MOVE_RIGHT_UP) || 
                    (blackboard_ptr_->command_mode == CMD_Command::MOVE_LEFT_UP) ||
                    (blackboard_ptr_->command_mode == CMD_Command::MOVE_LEFT_DOWN) ||
                    (blackboard_ptr_->command_mode == CMD_Command::LOOKAT) || 
                    (blackboard_ptr_->command_mode == CMD_Command::TARGET_TO_ENEMY_OUTPOST) ||
                    (blackboard_ptr_->command_mode == CMD_Command::TARGET_TO_ENEMY_PATRAL) ||
                    (blackboard_ptr_->command_mode == CMD_Command::TARGET_TO_HOME) ||
                    (blackboard_ptr_->command_mode == CMD_Command::TARGET_TO_MY_OUTPOST) ||
                    (blackboard_ptr_->command_mode == CMD_Command::TARGET_TO_MY_STEP) ||
                    (blackboard_ptr_->command_mode == CMD_Command::TARGET_TO_PATRAL_AREA);

    return check; 
}

bool Chassis_Switch::checktarget()
{
    bool check = (blackboard_ptr_->command_mode == CMD_Command::TARGET_TO_ENEMY_OUTPOST) ||
                 (blackboard_ptr_->command_mode == CMD_Command::TARGET_TO_ENEMY_PATRAL) ||
                 (blackboard_ptr_->command_mode == CMD_Command::TARGET_TO_HOME) ||
                 (blackboard_ptr_->command_mode == CMD_Command::TARGET_TO_MY_OUTPOST) ||
                 (blackboard_ptr_->command_mode == CMD_Command::TARGET_TO_MY_STEP) ||
                 (blackboard_ptr_->command_mode == CMD_Command::TARGET_TO_PATRAL_AREA);
    return check;
}

}