#pragma once
#include<ros/ros.h>
#include<ros/package.h>
#include<fstream>
#include<sstream>
#include<time.h>
#include<string.h>
#include"utility/utility.hpp"

class AutoAim_executor
{
    public:
        typedef std::shared_ptr<AutoAim_executor> Ptr;
        void initParam(ros::NodeHandle &nh_);
        void pub_autoaim_state(int state,int target_id);
        ros::Publisher status_publisher;
};