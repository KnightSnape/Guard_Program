#include"cmd_target.hpp"

int main(int argc,char **argv)
{
    ros::init(argc,argv,"cmd_target");
    ros::NodeHandle nh;
    auto Cmd_solver = std::make_shared<cmd_solver>(nh);
    ros::spin();
    return 0;
}