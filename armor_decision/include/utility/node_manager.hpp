#pragma once
#include"utility.hpp"

class ROSNodeManager
{
    public:
        ROSNodeManager();
        void ListNode();
        void restart_node();
        void update_deque(gary_msgs::RobotPosition pos);
        void average_deque();
        void checkiszerochassis(gary_msgs::RobotPosition &local_pos,bool &state);

        
    private:
        int max_deque_length;
        std::deque<gary_msgs::RobotPosition> robot_pos_sequence;
        Eigen::Vector3d average_pos;
        float average_yaw;
        std::string positioning_node_name;
        std::string positioning_launch_file;
        std::string navigating_node_name;
        std::string navigating_launch_name;
};