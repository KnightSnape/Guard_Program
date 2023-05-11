#include"node_manager.hpp"

ROSNodeManager::ROSNodeManager()
{
    positioning_node_name = "laser_mapping";
    navigating_node_name = "local_planner";
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
    
}