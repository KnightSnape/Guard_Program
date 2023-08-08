#pragma once
#include<Eigen/Eigen>
#include<opencv2/opencv.hpp>
#include<opencv2/core/eigen.hpp>
#include<vector>
#include<string>
#include<iostream>
#include<unordered_map>
#include<yaml-cpp/yaml.h>
#include<ros/ros.h>
#include<ros/package.h>
#include<ros/network.h>
#include<ros/xmlrpc_manager.h>
#include"xmlrpcpp/XmlRpc.h"
#include"std_msgs/Float64.h"
#include"nav_msgs/Odometry.h"
#include<tf/transform_datatypes.h>
#include<tf/transform_broadcaster.h>
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
#include"gary_msgs/DualLoopPID.h"
#include"gary_msgs/DR16Receiver.h"
#include"gary_msgs/DualLoopPIDWithFilter.h"
#include"gary_msgs/VisionModeSwitchRequest.h"
#include"gary_msgs/VisionModeSwitchResponse.h"
#include"guard_msgs/AutoaimState.h"

using namespace std;
using namespace cv;

#define PI 3.141592653589793238

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
//原地防守模式
struct StandBy_Control
{
    //是否启用
    bool is_standby;
    //一轮次最大控制时间
    double max_control_time;
    //剩余控制时间
    double remain_control_time;
};

//手控模式结构体
struct CMD_Control
{
    //是否启动手控
    bool is_control;
    //绝对向x方向移动
    float twist_x;
    //绝对向y方向移动
    float twist_y;
    //控制最大时间
    double max_control_time;
    //剩余最大时间
    double remain_control_time;
};
//发射控制结构体
struct Shoot_Control
{
    //是否可以发射
    bool is_shootable;
    //剩余弹量
    int remain_bullet;
    //控制最大时间
    double max_control_time;
    //剩余时间
    double remain_control_time;
};
//自瞄云台手优先瞄准控制结构体
struct AutoAim_Control
{
    //是否启用
    bool is_cmd;
    //目标
    int target_id;
    //控制最大时间
    double max_control_time;
    //剩余时间
    double remain_control_time;
};

struct AutoAim_State_Control
{
    //是否启用
    bool is_cmd;
    //控制最大时间
    double max_control_time;
    //剩余时间
    double remain_control_time;
};


enum class CMD_Command
{
    INITIAL = 0,//无变化
    LOOKAT = 1,//向某个方向看
    REPOS = 2,//刷新位置，重启定位导航系统
    PRIORITY_ATTACK = 3,//优先攻击target单位
    STOP_MOVING = 4,//立即停止移动，持续一段时间
    STOP_SHOOT = 5,//立即停止射击，持续一段时间
    MOVE_UP = 6,//强制向小地图上方移动
    MOVE_DOWN = 7,//强制向小地图下方移动
    MOVE_LEFT = 8,//强制向小地图左方移动
    MOVE_RIGHT = 9,//强制向小地图右方移动
    MOVE_TARGET = 10,//向某个方向移动
    MOVE_UP_ABS = 11,//强制向前
    MOVE_DOWN_ABS = 12,//强制向后
    MOVE_LEFT_ABS = 13,//强制向左
    MOVE_RIGHT_ABS = 14,//强制向右
    TARGET_TO_HOME = 15,//初始巡逻区位置
    TARGET_TO_PATRAL_AREA = 16,//狙击点巡逻区位置
    TARGET_TO_MY_OUTPOST = 17,//我方前哨战前
    TARGET_TO_MY_STEP = 18,//我方台阶上
    TARGET_TO_ENEMY_OUTPOST = 19,//敌方前哨战前
    TARGET_TO_ENEMY_PATRAL = 20,//敌方巡逻区前
    START_ROTATE = 21,//开启小陀螺
    STOP_ROTATE = 22,//停止小陀螺
    MOVE_RIGHT_UP = 23,//强制向右上方移动
    MOVE_RIGHT_DOWN = 24,//强制向右下方移动
    MOVE_LEFT_UP = 25,//强制向左上方移动
    MOVE_LEFT_DOWN = 26,//强制向左下方移动
    TRANSFORM_TO_HIGH_SPEED = 27,//强制转换状态为高角度扫描
    TRANSFORM_TO_SCANNING = 28,//强制转换状态为360扫描
};

//底盘状态
enum class Chassis_Mode 
{
    INITIAL = 0,//无变化，静止
    STANDBY = 1,//原地
    LISTEN_CAPTAIN = 2,//听从云台手指令
    FOLLOW_GIMBAL = 3,//将跟随云台，但不作为指令发送。当目标可被击杀且逐渐变远的时候，启用追击模式
    FREE_CAPTAIN = 4,//自动判断已有信息后，选择点位进行导航
    NAVIGATING = 5,//导航进行中，此时只有云台手指令才能打断
    CAPTAIN_CONTROL = 6,//云台手控制模式
    ROTATE = 7,//小陀螺模式
}; 
//云台状态
enum class Gimbal_Mode
{
    NO_FORCE = 0,//无力，不发送
    STEADY = 1,//无变化，静止(但可以测试自瞄，然而不能发弹丸)
    LOW_SPEED = 2,//低速平转巡航，存在角度控制
    HIGH_SPEED = 3,//高度平转巡航
    SCANNING = 4,//360度中速巡航
    WARN_CAPTAIN = 5,//优先级高于跟随自瞄目标，云台一定转向云台手发送的方向
    FOLLOW_AUTOAIM = 6,//跟随自瞄目标
};

enum class AutoAim_Mode
{
    INITIAL = 0,//无变化
    OUTPOST = 1,//以前哨站为优先
    KILL_TARGET = 2,//必单杀目标锁定
    NORMAL = 3,//常规目标锁定
    LISTEN_CAPTAIN = 4,//云台手目标锁定
};