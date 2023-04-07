#include"navigation_node.hpp"
int main(int argc,char **argv)
{
    ros::init(argc,argv,"navigation_node");
    std::shared_ptr<navigation::Navigation_Solver> Ptr = std::make_shared<navigation::Navigation_Solver>(argc,argv);
    ros::spin();
    ros::shutdown();
    return 0;
}