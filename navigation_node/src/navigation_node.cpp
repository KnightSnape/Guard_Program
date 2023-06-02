#include"navigation.hpp"

int main(int argc,char **argv)
{
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::init(argc,argv,"navigation_node");
    navigation nav;
    return 0;
}