#pragma once
#include<opencv2/opencv.hpp>
#include<Eigen/Eigen>
#include<vector>
#include<string>
#include<iostream>
#include<unordered_map>
#include"gary_msgs/AutoAIM.h"
#include"gary_msgs/RobotHP.h"
#include"gary_msgs/BulletRemaining.h"
#include"gary_msgs/GameStatus.h"
#include"gary_msgs/GameResult.h"
#include"gary_msgs/RobotBuff.h"
#include"gary_msgs/RobotPosition.h"
#include"gary_msgs/RobotStatus.h"
#include"gary_msgs/ClientCommand.h"
#include"gary_msgs/ClientReceive.h"
#include"gary_msgs/VisionModeSwitch.h"
#include"gary_msgs/VisionModeSwitchRequest.h"
#include"gary_msgs/VisionModeSwitchResponse.h"

using namespace std;
using namespace cv;

struct Graph_Node
{
    //node的ID
    int node_id;
    //node的名字
    string node_name;
    //node对应点的小地图坐标
    Point node_place;
    //node对应的区域
    vector<Point> node_area;
    //这里储存下一个node的map
    vector<pair<int,int>> next_node_map;
};

//联盟赛使用的状态
enum class Add_Blood_State
{
    INITIAL = 0,//基本的初始化状态
    EXCELLENT = 1,//状态良好，在比赛开始时变换成该状态
    FIND_D_TAG = 2,//血量不足，寻找D_tag(不快速的转动云台，直到找到D_tag并稳定识别之后才能切换)
    GOTO_D_TAG = 3,//找到D_tag之后，会一直保持这个状态直到检测到自己补血。
    RECOVERING = 4,//这个状态下会一直检测自己的血量状态，直到满血之后才会搜寻tag位置
    FIND_A_TAG = 5,//原理和FIND_D_TAG相同
    GOTO_A_TAG = 6,//原理和GOTO_D_TAG相同(停下来切换是依据视觉解算距离判断防止视野过小)
};
//底盘状态
enum class Chassis_Mode 
{
    INITIAL = 0,//无变化，静止
    STANDBY = 1,//原地
    LISTEN_CAPTAIN = 2,//听从云台手指令
    FOLLOW_GIMBAL = 3,//将跟随云台，但不作为指令发送。当目标可被击杀且逐渐变远的时候，启用追击模式
    FREE_CAPTAIN = 4,//自动判断已有信息后，选择点位进行导航
}; 
//云台状态
enum class Gimbal_Mode
{
    INITIAL = 0,//无变化，静止(但可以测试自瞄，然而不能发弹丸)
    LOW_SPEED = 1,//低速平转巡航，存在角度控制
    SCANNING = 2,//360度中速巡航
    WARN_CAPTAIN = 3,//优先级高于跟随自瞄目标，云台一定转向云台手发送的方向
    FOLLOW_AUTOAIM = 4,//跟随自瞄目标
};

enum class AutoAim_Mode
{
    INITIAL = 0,//无变化
    OUTPOST = 1,//以前哨站为优先
    KILL_TARGET = 2,//必单杀目标锁定
    NORMAL = 3,//常规目标锁定
    A_TAG = 4,//A的TAG锁定
    D_TAG = 5,//D的TAG锁定
};