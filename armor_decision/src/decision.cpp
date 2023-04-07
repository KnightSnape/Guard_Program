#include<ros/ros.h>
#include"decision.hpp"
namespace decision_tree
{
    decision_node::decision_node()
    {
        ros::NodeHandle nh;

        loop_rate_ = 5;//10Hz or 30Hz, 控制处理频率

        //导入新地图
        blackboard_ = std::make_shared<Blackboard>();
        chassis_exe_ = std::make_shared<Chassis_executor>();
        gimbal_exe_ = std::make_shared<Gimbal_executor>();
        log_exe_ = std::make_shared<Log_executor>();
        warm_exe_ = std::make_shared<Warn_executor>();

        //根节点需要用时间的Selector
        root_node = new Race_Choose("robot_decision",0,blackboard_,chassis_exe_,gimbal_exe_,log_exe_,warm_exe_);
        //选择联盟赛行为树还是对抗赛行为树

        LeagueTreeBuild();
        MatchTreeBuild();
        //TODO:架构完成之后编写log文件
        ROS_INFO("add_Tree Complete");
        decision_thread = std::thread(&decision_node::ExecuteLoop,this);
        decision_thread_running_ = true;
    }

    void decision_node::ExecuteLoop()
    {
        ros::Rate loop_rate(loop_rate_);
        int cnt = 0;
        while(decision_thread_running_)
        {
            root_node->Run();
            cnt++;
            if(cnt >= loop_rate_)
            {
                cnt = 0;
            }
            loop_rate.sleep();
        }
    }

    //联盟赛行为树搭建
    void decision_node::LeagueTreeBuild()
    {
        
    }

    //对抗赛行为树搭建
    void decision_node::MatchTreeBuild()
    {

    }
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"robot_decision_node");
    ros::NodeHandle ros_nh;
    decision_tree::decision_node devision;
    ros::spin();
    return 0;
}