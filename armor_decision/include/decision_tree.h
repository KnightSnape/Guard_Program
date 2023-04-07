#pragma once
#include<chrono>
#include<random>
#include<map>
#include<functional>
#include<algorithm>
#include<iostream>
#include<Eigen/Eigen>
#include<string>
#include"executor/chassis_executor.hpp"
#include"executor/log_executor.hpp"
#include"executor/warn_executor.hpp"
#include"executor/gimbal_executor.hpp"
#include"utility/blackboard.hpp"
using namespace std;
namespace decision_tree
{
    enum class BehaviorState
    {
        SUCCESS,
        FAILURE
    };
    class TreeNode
    {
        public:
            TreeNode(){}
            TreeNode(
                std::string name,
                int level,
                const Blackboard::Ptr& blackboard,
                const Chassis_executor::Ptr &chassis_executor,
                const Gimbal_executor::Ptr &gimbal_executor,
                const Log_executor::Ptr &log_executor,
                const Warn_executor::Ptr &warn_executor):
                name_(name),
                level_(level),
                blackboard_ptr_(blackboard),
                chassis_exe_ptr_(chassis_executor),
                gimbal_exe_ptr_(gimbal_executor),
                log_exe_ptr_(log_executor),
                warn_exe_ptr_(warn_executor)
            {}
            virtual ~TreeNode() = default;
            virtual BehaviorState Update() = 0;
            virtual void print() = 0;
            virtual void print_tree() = 0;
            BehaviorState Run()
            {
                behavior_state_ = Update();
                return behavior_state_;
            }
            Blackboard::Ptr blackboard_ptr_;
            Chassis_executor::Ptr chassis_exe_ptr_;
            Gimbal_executor::Ptr gimbal_exe_ptr_;
            Log_executor::Ptr log_exe_ptr_;
            Warn_executor::Ptr warn_exe_ptr_;
            int level_;
            string name_;
            BehaviorState behavior_state_;

    };

    class SequenceNode : public TreeNode
    {
        public:
            SequenceNode():TreeNode::TreeNode(){};
            SequenceNode(std::string name,
                        int level,
                        const Blackboard::Ptr &blackboard,
                        const Chassis_executor::Ptr &chassis_executor,
                        const Gimbal_executor::Ptr &gimbal_executor,
                        const Log_executor::Ptr &log_executor,
                        const Warn_executor::Ptr &warn_executor):
                        TreeNode::TreeNode(name,level,blackboard,chassis_executor,gimbal_executor,log_executor,warn_executor){}
            virtual ~SequenceNode() = default;
            //运行一次函数增加一个孩子
            void addChild(TreeNode* child_node_ptr)
            {
                child_node_ptr_list_.push_back(child_node_ptr);
            }
            //遍历孩子，只有所有的全部成功才能算作成功
            virtual BehaviorState Update()
            {
                if(child_node_ptr_list_.size() == 0)
                {
                    return BehaviorState::FAILURE;
                }
                else
                {
                    for(int i = 0;i < child_node_ptr_list_.size();i++)
                    {
                        if(child_node_ptr_list_[i]->Run() == BehaviorState::FAILURE)
                        {
                            return BehaviorState::FAILURE;
                        }
                    }
                    return BehaviorState::SUCCESS;
                }
            }
            void print()
            {
                std::stringstream str;
                for(int i = 0; i < level_; i++)
                {
                    str << "  ";
                }
                str << "+ " << name_.data();
                log_exe_ptr_->print(str);
            }
            void print_tree()
            {
                print();
                for(int i = 0; i < child_node_ptr_list_.size(); i++)
                {
                    child_node_ptr_list_[i]->print_tree();
                }
            }     
            std::vector<TreeNode*> child_node_ptr_list_;
    };

    class SelectorNode : public TreeNode
    {
        public:
            SelectorNode():TreeNode::TreeNode(){};
            SelectorNode(std::string name,
                        int level,
                        const Blackboard::Ptr &blackboard,
                        const Chassis_executor::Ptr &chassis_executor,
                        const Gimbal_executor::Ptr &gimbal_executor,
                        const Log_executor::Ptr &log_executor,
                        const Warn_executor::Ptr &warn_executor):
                        TreeNode::TreeNode(name,level,blackboard,chassis_executor,gimbal_executor,log_executor,warn_executor){}
            virtual ~SelectorNode() = default;
            void addChild(TreeNode* child_node_ptr)
            {
                child_node_ptr_list_.push_back(child_node_ptr);
            }
            //遍历孩子，在该顺序下如果成功，则返回成功
            virtual BehaviorState Update()
            {
                if(child_node_ptr_list_.size() == 0)
                {
                    return BehaviorState::FAILURE;
                }
                else
                {
                    for(int i = 0;i < child_node_ptr_list_.size();i++)
                    {
                        if(child_node_ptr_list_[i]->Run() == BehaviorState::SUCCESS)
                        {
                            return BehaviorState::SUCCESS;
                        }
                    }
                    return BehaviorState::FAILURE;
                }
            }
            void print()
            {
                std::stringstream str;
                for(int i = 0; i < level_; i++)
                {
                    str << "  ";
                }
                str << "+ " << name_.data();
                log_exe_ptr_->print(str);
            }
            void print_tree()
            {
                print();
                for(int i = 0; i < child_node_ptr_list_.size(); i++)
                {
                    child_node_ptr_list_[i]->print_tree();
                }
            }
            std::vector<TreeNode*> child_node_ptr_list_;

    };

    class ActionNode : public TreeNode
    {
        public:
            ActionNode():TreeNode::TreeNode(){}
            ActionNode(std::string name,
                        int level,
                        const Blackboard::Ptr &blackboard,
                        const Chassis_executor::Ptr &chassis_executor,
                        const Gimbal_executor::Ptr &gimbal_executor,
                        const Log_executor::Ptr &log_executor,
                        const Warn_executor::Ptr &warn_executor):
                        TreeNode::TreeNode(name,level,blackboard,chassis_executor,gimbal_executor,log_executor,warn_executor){}
            virtual ~ActionNode() = default;
            void print()
            {
                std::stringstream str;
                for(int i = 0; i < level_; i++)
                {
                    str << "  ";
                }
                str << "+ " << name_.data();
                log_exe_ptr_->print(str);
            }
            void print_tree()
            {
                print();
            }
    };
}

