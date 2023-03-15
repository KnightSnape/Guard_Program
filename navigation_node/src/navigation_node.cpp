#include"navigation_node.hpp"
int main(int argc,char **argv)
{
    ros::init(argc,argv,"navigation_node");
    std::shared_ptr<Navigation_Solver> Ptr = std::make_shared<Navigation_Solver>(argc,argv);
    ros::spin();
    ros::shutdown();
    return 0;
}