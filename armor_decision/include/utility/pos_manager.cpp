#include"pos_manager.hpp"

Pos_Manager::Pos_Manager()
{
    max_rotate_angle = 10.0 / 180.0 * PI;
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
    map_pos.x() = ((int)(pw.x() / 28.0 * 840)) + pos_x_zero;
    map_pos.y() = (450 - (int)(pw.y() / 15.0 * 450)) + pos_y_zero;
    return map_pos;
}

Eigen::Vector3d Pos_Manager::map_to_world(const Eigen::Vector2i &pm)
{
    //TODO(Knight):update transform
    Eigen::Vector3d world_pos;
    world_pos.x() = (pm.x() - pos_x_zero) / 840.0 * 28;
    world_pos.y() = ((pm.y() - pos_y_zero) / 450.0 * 15);
    world_pos.z() = 0.0;
    return world_pos;
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

float Pos_Manager::get_yaw()
{
    Eigen::Vector3d eular_angle = this->guard_rotate_q.matrix().eulerAngles(2,1,0);
    return eular_angle[2];
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
        if(now_angle < -0.5 * PI)
            need_yawangle = -1.5 * PI;
        else
            need_yawangle = 0.5 * PI;
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
        if(now_angle > 0.5 * PI)
            need_yawangle = 0.5 * PI;
        else 
            need_yawangle = -0.5 * PI;
    }
    ROS_DEBUG("now_angle is:%f,need_angle is:%f");
    if(std::abs(now_angle - need_yawangle) > 1.0 / 180.0 * PI)
    {
        yaw_angle = (std::abs(now_angle - need_yawangle) > max_rotate_angle) ? 
                    (getsign(need_yawangle,now_angle) * max_rotate_angle) : 
                    (getsign(need_yawangle,now_angle) * std::abs(now_angle - need_yawangle) / 2);
        twist_x = 0.0;
        twist_y = 0.0;
    }
    else
    {
        yaw_angle = 0;
        twist_x = 0.3;
        twist_y = 0;
    }
}

