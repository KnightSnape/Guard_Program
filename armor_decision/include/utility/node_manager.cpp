#include"node_manager.hpp"

ROSNodeManager::ROSNodeManager()
{
    positioning_node_name = "laser_mapping";
    navigating_node_name = "local_planner";
    max_deque_length = 10;
    ListNode();
}

void ROSNodeManager::ListNode()
{
    std::vector<std::string> nodes;
    if(!ros::master::getNodes(nodes))
    {
        ROS_ERROR("Failed to get node list");
        return;
    }
}

void ROSNodeManager::restart_node()
{
    std::string bash_path =  "source " + ros::package::getPath("gary_msgs") + "../../devel/setup.bash";
    system(bash_path.c_str());
    
}

void ROSNodeManager::update_deque(gary_msgs::RobotPosition pos)
{
    this->robot_pos_sequence.push_back(pos);
    if(robot_pos_sequence.size() > max_deque_length)
        this->robot_pos_sequence.pop_front();
}

void ROSNodeManager::average_deque()
{
    Eigen::Vector3d sum_pos = Eigen::Vector3d::Zero();
    float sum_yaw = 0;
    for(int i=0;i<robot_pos_sequence.size();i++)
    {
        sum_pos.x() += robot_pos_sequence[i].x;
        sum_pos.y() += robot_pos_sequence[i].y;
        sum_pos.z() += robot_pos_sequence[i].z;
        sum_yaw += robot_pos_sequence[i].yaw;
    }
    average_pos = sum_pos / robot_pos_sequence.size();
    average_yaw = sum_yaw / robot_pos_sequence.size();
}

void ROSNodeManager::checkiszerochassis(gary_msgs::RobotPosition &local_pos,bool &state)
{
    if(average_yaw < 2.0 * PI / 180.0 && average_yaw > -2.0 * PI / 180.0)
    {
        local_pos.x = average_pos.x();
        local_pos.y = average_pos.y();
        local_pos.z = average_pos.z();
        local_pos.yaw = average_yaw;
        state = true;
    }
    else
        state = false;
}

