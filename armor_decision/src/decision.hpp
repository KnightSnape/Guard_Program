#pragma once
#include<ros/ros.h>
#include<thread>
#include<mutex>
#include"Timer.h"
#include"decision_tree.h"
#include"blackboard.hpp"

#include"log_executor.hpp"
#include"warn_executor.hpp"
#include"chassis_executor.hpp"

#include"add_blood.hpp"
#include"attack_enemy.hpp"
#include"attack_outpost.hpp"
#include"chase.hpp"
#include"defend_base.hpp"
#include"defend_outpost.hpp"
#include"game_start.hpp"
#include"help_others.hpp"
#include"midplaceoccpuy.hpp"
#include"protect_hero.hpp"
#include"retreat.hpp"
#include"calculate_point.hpp"
#include"time_get.hpp"
#include"goto_place.hpp"
#include"race_choose.hpp"

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
        private:
            SelectorNode* root_node;
            Blackboard::Ptr blackboard_;
            Chassis_executor::Ptr chassis_exe_;
            Log_executor::Ptr log_exe_;
            Warn_executor::Ptr warm_exe_;
            cv::Mat map_;
            bool visualize_flag_;
            std::thread decision_thread;
            bool decision_thread_running_;
            int loop_rate_;
    };
}