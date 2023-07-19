#include"pos_manager.hpp"

Pos_Manager::Pos_Manager()
{
    max_rotate_angle = 40.0 / 180.0 * PI;
    this->pos_x_zero = 170;
    this->pos_y_zero = 225;
}

Eigen::Vector2i Pos_Manager::inverse_point(const Eigen::Vector2i &p)
{
    Eigen::Vector2i inverse_p;
    inverse_p.x() = 840 - p.x();
    inverse_p.y() = 450 - p.y();
    return inverse_p;
}

Eigen::Vector2i Pos_Manager::world_to_map(const Eigen::Vector3d &pw)
{
    Eigen::Vector2i map_pos;
    map_pos.x() = ((int)(pw.x() / 28.0 * 840));
    map_pos.y() = ((int)(pw.y() / 15.0 * 450));
    return map_pos;
}

Eigen::Vector3d Pos_Manager::map_to_world(const Eigen::Vector2i &pm)
{
    //TODO(Knight):update transform
    Eigen::Vector3d world_pos;
    world_pos.x() = (pm.y() - pos_y_zero) / 450.0 * 15;
    world_pos.y() = ((pm.x() - pos_x_zero) / 840.0 * 28);
    world_pos.z() = 0.0;
    return world_pos;
}

Eigen::Vector2i Pos_Manager::robot_to_map(const Eigen::Vector3d &pw)
{
    Eigen::Vector2i map_pos;
    map_pos.x() = (int)(pw.y() / 28.0 * 840) + pos_x_zero;
    map_pos.y() = (int)(pw.x() / 15.0 * 450) + pos_y_zero;
    return map_pos;
}

geometry_msgs::PointStamped Pos_Manager::transfer_to_msg(const Eigen::Vector3d &pw)
{
    geometry_msgs::PointStamped point_msg;
    point_msg.point.x = pw.x();
    point_msg.point.y = pw.y();
    point_msg.point.z = pw.z();
    return point_msg;
}

void Pos_Manager::set_guard_world_pos(const Eigen::Vector3d &point)
{
    this->guard_world_pos_now = point;
}

void Pos_Manager::set_guard_world_pos(const nav_msgs::Odometry &odom)
{
    this->guard_world_pos_now.x() = odom.pose.pose.position.x;
    this->guard_world_pos_now.y() = odom.pose.pose.position.y;
    this->guard_world_pos_now.z() = odom.pose.pose.position.z;
}

void Pos_Manager::set_guard_rotate_q(const Eigen::Quaterniond &q)
{
    this->guard_rotate_q = q;
}

void Pos_Manager::set_guard_rotate_q(const nav_msgs::Odometry &odom)
{
    this->guard_rotate_q.w() = odom.pose.pose.orientation.w;
    this->guard_rotate_q.x() = odom.pose.pose.orientation.x;
    this->guard_rotate_q.y() = odom.pose.pose.orientation.y;
    this->guard_rotate_q.z() = odom.pose.pose.orientation.z;
}

double Pos_Manager::get_yaw()
{
    return this->yaw;
}

Eigen::Vector3d Pos_Manager::get_guard_world_pos()
{
    return this->guard_world_pos_now;
}

Eigen::Quaterniond Pos_Manager::get_guard_rotate_q()
{
    return this->guard_rotate_q;
}

void Pos_Manager::set_pos_x_zero(int pos_x_zero)
{
    this->pos_x_zero = pos_x_zero;
}

void Pos_Manager::set_pos_y_zero(int pos_y_zero)
{
    this->pos_y_zero = pos_y_zero;
}

void Pos_Manager::setRPY()
{
    tf::Matrix3x3(tf::Quaternion(guard_rotate_q.x(), guard_rotate_q.y(), guard_rotate_q.z(), guard_rotate_q.w())).getRPY(roll, pitch, yaw);
}

void Pos_Manager::rotate_angle(float angle)
{
    angle = angle / 180.0 * PI;
    Eigen::Vector3d eular_angle = this->guard_rotate_q.matrix().eulerAngles(2,1,0);
    eular_angle.z() = eular_angle.z() + angle;//需要测试
    this->guard_rotate_q = Eigen::AngleAxisd(eular_angle[0],Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(eular_angle[1],Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(eular_angle[2],Eigen::Vector3d::UnitZ());
}

void Pos_Manager::speed_process(float &twist_x,float &twist_y,int mode,float &yaw_angle)
{
    if(mode == 0)
        return;
    float now_angle = get_yaw();
    float need_yawangle;
    if(mode == 1)
    {
        need_yawangle = 0;
    }
    else if(mode == 2)
    {
        if(now_angle > 0.5 * PI)
            need_yawangle = 1.5 * PI;
        else
            need_yawangle = -0.5 * PI;
    }
    else if(mode == 3)
    {
        if(now_angle < 0)
            need_yawangle = -PI;
        else
            need_yawangle = PI;
    }
    else if(mode == 4)
    {
        if(now_angle < -0.5 * PI)
            need_yawangle = -1.5 * PI;
        else 
            need_yawangle = 0.5 * PI;
    }
    else if(mode == 5)
    {
        if(now_angle < -0.75 * PI)
            need_yawangle = -1.75 * PI;
        else    
            need_yawangle = 0.25 * PI;
    }
    else if(mode == 6)
    {
        if(now_angle < -0.25 * PI)
            need_yawangle = -1.25 * PI;
        else
            need_yawangle = 0.75 * PI;
    }
    else if(mode == 7)
    {
        if(now_angle > 0.75 * PI)
            need_yawangle = 1.75 * PI;
        else 
            need_yawangle = -0.25 * PI;
    }
    else if(mode == 8)
    {
        if(now_angle > 0.25 * PI)
            need_yawangle = 1.25 * PI;
        else 
            need_yawangle = -0.75 * PI;
    }
    ROS_DEBUG("now_angle is:%f,need_angle is:%f");
    if(std::abs(now_angle - need_yawangle) > 6.0 / 180.0 * PI)
    {
        yaw_angle = (getsign(need_yawangle,now_angle) * max_rotate_angle);
        twist_x = 0.0;
        twist_y = 0.0;
    }
    else
    {
        yaw_angle = 0;
        twist_x = 0.35;
        twist_y = 0;
    }
}

void Pos_Manager::rotate_process(float& yaw_angle,const float& target_yaw_angle)
{
    float now_angle = get_yaw();
    if(std::abs(now_angle - target_yaw_angle) > 30.0 / 180.0 * PI)
    {
        yaw_angle = (getsign(target_yaw_angle,now_angle) * max_rotate_angle) * 10;
    }
    else if(std::abs(now_angle - target_yaw_angle) > 15.0 / 180.0 * PI)
    {
        yaw_angle = (getsign(target_yaw_angle,now_angle) * max_rotate_angle) * 3;
    }
    else
    {
        yaw_angle = 0;
    }
}

double Pos_Manager::calculate_relative_angle(const Eigen::Vector2i &a1, const Eigen::Vector2i &a2)
{
   Eigen::Vector2i delta_pos = a2 - a1;
   Eigen::Vector2i x_pos{1,0};
   double dot = (double)(delta_pos.x() * x_pos.x() + delta_pos.y() * x_pos.y());
   double cross = (double)(x_pos.x() * delta_pos.y() - x_pos.y() * delta_pos.x());
   double angle = atan2(cross,dot);
   return angle;
}

double Pos_Manager::calculate_relative_angle(const Eigen::Vector2d &a1, const Eigen::Vector2d &a2)
{
   Eigen::Vector2d delta_pos = a2 - a1;
   Eigen::Vector2d x_pos{1,0};
   double dot = (double)(delta_pos.x() * x_pos.x() + delta_pos.y() * x_pos.y());
   double cross = (double)(x_pos.x() * delta_pos.y() - x_pos.y() * delta_pos.x());
   double angle = atan2(cross,dot);
   return angle;    
}

bool Pos_Manager::navigation_process(bool is_navigation_signal,Eigen::Vector3d target_point,Eigen::Vector3d self_point,geometry_msgs::Twist& twist_msg)
{
    if(!is_navigation_signal)
        return false;
    Eigen::Vector2d target_pos_2d{target_point[0],target_point[1]};
    Eigen::Vector2d self_pos_2d{self_point[0],self_point[1]};

    Eigen::Vector2d delta_pos = target_pos_2d - self_pos_2d;
    Eigen::Vector2d x_pos{1,0};

    double norm = delta_pos.norm();
    yaw = yaw + (0.5 * M_PI);
    if(yaw > M_PI)
        yaw -= 2 * M_PI;

    if(norm == 0)
        return false;

    double angle = calculate_relative_angle(target_pos_2d,self_pos_2d);
    double delta_angle = yaw - angle;
    if(delta_angle < -M_PI)
        delta_angle += 2 * M_PI;
    if(delta_angle > M_PI)
        delta_angle -= 2 * M_PI;
    bool use_final_angle = false;
    double final_angle = 0.0;
    Eigen::Vector2d e_pos2d{cos(yaw),sin(yaw)};

    double cos_angle_temp = (delta_pos.x() * e_pos2d.x() + delta_pos.y() * e_pos2d.y()) / (delta_pos.norm() * e_pos2d.norm());

    if(delta_angle > 5.0 / 180 * M_PI)
    {
        final_angle = 20.0 / 180 * M_PI;
        use_final_angle = true;
    }
    if(delta_angle < -5.0 / 180 * M_PI)
    {
        final_angle = -20.0 / 180 * M_PI;
        use_final_angle = true;
    }

    Eigen::Vector3d final_twist = Eigen::Vector3d::Zero();

    if(!use_final_angle)
    {
        double final_speed = 0;
        if(norm > 2)
            final_speed = 1.0;
        else if(norm > 0.06 && norm <= 4)
            final_speed = 0.5;
        else 
            final_speed = 0;
        final_twist.x() = final_speed;
        final_twist.y() = 0.0;
        final_twist.z() = 0.0;
    }

    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = final_angle;
    twist_msg.linear.x = final_twist.x();
    twist_msg.linear.y = final_twist.y();
    twist_msg.linear.z = final_twist.z();

    return true;
}