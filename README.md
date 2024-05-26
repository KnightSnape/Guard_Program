# Guard_Program

***Author: KnightSnape***

此项目为哨兵机器人的决策和导航模块

## Background

开源有点晚了，这个方案实际上也是在去年五月份的一个突发奇想，由于Knight有太多的任务压身没有办法对导航本身做太深的研究，故用贪心策略来解决决策+导航问题，即将重要位置进行场地划分，将导航问题转化为直线问题而不考虑点云本身，也就是说只要对地图的预解析正确，即可做全局路径规划。

## Environment

- ros-noetic
- Qt5
- OpenCV
- Eigen
- yaml-cpp

## Build 

```
source /opt/ros/noetic/setup.bash
catkin_make 
```
## Conclusion

- 本项目是RM2023的解决方案，对于2024已不适用(本身为半自动准备，因为我相信我的教练)，ros-noetic版本的项目也不再适用。
- 后续的自决策由我的学弟学妹维护，也是基于这个架构实现的
- 如果你对该项目有兴趣，可以联系我们的团队

## Contact 

- KnightSnape: Yile.Xu21@student.xjtlu.edu.cn
- MyTeam: TeamGMaster@xjtlu.edu.cn






