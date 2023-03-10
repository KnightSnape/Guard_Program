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
        log_exe_ = std::make_shared<Log_executor>();
        warm_exe_ = std::make_shared<Warn_executor>();

        //根节点需要用时间的Selector
        root_node = new SelectorNode("robot_decision",0,blackboard_,chassis_exe_,log_exe_,warm_exe_);
        //选择联盟赛行为树还是对抗赛行为树
        Race_Choose* race_choose = new Race_Choose("race_choose",1,blackboard_,chassis_exe_,log_exe_,warm_exe_);
        root_node->addChild(race_choose);     
        //联盟赛或对抗赛的根节点
        SelectorNode* League = new SelectorNode("league",2,blackboard_,chassis_exe_,log_exe_,warm_exe_);
        SelectorNode* Match = new SelectorNode("match",2,blackboard_,chassis_exe_,log_exe_,warm_exe_);
        //联盟赛行为树准备搭建
        //RMUL
        race_choose->addChild(League);
        //RMUC
        race_choose->addChild(Match);

        //游戏开始节点(可能还需要改，需要考虑里程寄问题)
        GameStart* game_start_node = new GameStart("gamestart",3,blackboard_,chassis_exe_,log_exe_,warm_exe_);
        
        League->addChild(game_start_node);
        Match->addChild(game_start_node);

        //下面是联盟赛的行为树子节点
        SelectorNode* first_four_minute_node = new SelectorNode("first_four_minute",3,blackboard_,chassis_exe_,log_exe_,warm_exe_);
        League->addChild(first_four_minute_node);

        //第一个需要有需要回血检测，在满足相应的条件下进行回血行为
        Add_Blood* add_blood_node = new Add_Blood("add_bloods",4,blackboard_,chassis_exe_,log_exe_,warm_exe_);
        //而在对于没有回血流程中，它会原地陀螺并且攻击已经看到的目标，若没有就缓慢扫视
        AttackEnemy* attack_enemy_node = new AttackEnemy("attack_enemys",4,blackboard_,chassis_exe_,log_exe_,warm_exe_);
        
        first_four_minute_node->addChild(add_blood_node);
        first_four_minute_node->addChild(attack_enemy_node);

        AttackEnemy* final_attack = new AttackEnemy("final_attack",3,blackboard_,chassis_exe_,log_exe_,warm_exe_);

        League->addChild(final_attack);
         
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
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"robot_decision_node");
    ros::NodeHandle ros_nh;
    decision_tree::decision_node devision;
    ros::spin();
    return 0;
}