#pragma once
#include<ros/ros.h>
#include<thread>
#include<mutex>
#include"Timer.h"
#include"decision_tree.h"
#include"utility/blackboard.hpp"

#include"log_executor.hpp"
#include"warn_executor.hpp"
#include"chassis_executor.hpp"
#include"gimbal_executor.hpp"

#include"race_choose.hpp"
#include"game_start.hpp"

namespace decision_tree
{
    class decision_node
    {
        public:
            decision_node();
            ~decision_node()
            {
                if(decision_thread.joinable())
                {
                    decision_thread_running_ = false;
                    decision_thread.join();
                }
                delete root_node;
            }
            void ExecuteLoop();
            void visualize();
            void LeagueTreeBuild();
            void MatchTreeBuild();
        private:
            Race_Choose* root_node;
            SequenceNode* League_node;
            SequenceNode* Match_node;
            Blackboard::Ptr blackboard_;
            Chassis_executor::Ptr chassis_exe_;
            Gimbal_executor::Ptr gimbal_exe_;
            Log_executor::Ptr log_exe_;
            Warn_executor::Ptr warm_exe_;
            cv::Mat map_;
            bool visualize_flag_;
            std::thread decision_thread;
            bool decision_thread_running_;
            int loop_rate_;
    };
}