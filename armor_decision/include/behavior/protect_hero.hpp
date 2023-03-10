#pragma once 
#include"decision_tree.h"
#include"chassis_executor.hpp"
#include"log_executor.hpp"
#include"warn_executor.hpp"

namespace decision_tree
{
    class ProtectHero : public ActionNode
    {
        public:
            ProtectHero(std::string name,
                        int level,
                        const Blackboard::Ptr &blackboard,
                        const Chassis_executor::Ptr &chassis_executor,
                        const Log_executor::Ptr &log_executor,
                        const Warn_executor::Ptr &warn_executor):
                        ActionNode::ActionNode(name,level,blackboard,chassis_executor,log_executor,warn_executor){}

            BehaviorState Update()
            {
                
            }
        private:
    };
}