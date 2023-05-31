#include"navigation.hpp"

int main(int argc,char **argv)
{

    ros::init(argc,argv,"navigation_node");
    navigation::Ptr nav = std::make_shared<navigation>();
    nav->main_process();
    ros::spin();
    return 0;
}