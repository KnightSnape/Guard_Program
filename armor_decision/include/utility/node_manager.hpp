#pragma once
#include"utility.hpp"

class ROSNodeManager
{
    public:
        ROSNodeManager();
        void ListNode();
        void restart_node();
        
    private:
        std::string positioning_node_name;
        std::string positioning_launch_file;
        std::string navigating_node_name;
        std::string navigating_launch_name;
};