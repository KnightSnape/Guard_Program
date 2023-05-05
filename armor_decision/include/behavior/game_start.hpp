#pragma once 
#include"decision_tree.h"
#include"chassis_executor.hpp"
#include"gimbal_executor.hpp"
#include"log_executor.hpp"
#include"autoaim_executor.hpp"
#include"gimbal_executor.hpp"

namespace decision_tree
{
    class GameStart : public ActionNode
    {
        public:
            GameStart(std::string name,
                        int level,
                        const Blackboard::Ptr &blackboard,
                        const Chassis_executor::Ptr &chassis_executor,
                        const Gimbal_executor::Ptr &gimbal_executor,
                        const Log_executor::Ptr &log_executor,
                        const AutoAim_executor::Ptr &autoaim_executor);
            BehaviorState Update();
        private:
            bool is_start;
    };
}