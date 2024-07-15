# aloam_note

学习ALOAM，添加注释代码
为了同步

# 文件结构

```
├── CMakeLists.txt - CMake配置文件
├── docker - 
    ├── Dockerfile - 
    ├── Makefile - 
    └── run.sh - 
├── include - 工具类及一些常用的定义
    └── aloam_velodyne - 该项目名称
        ├── common.h - 主要是数学角度转换，cos,sin等
        └── tic_toc.h - 计算处理时间，计时用的
├── launch - 启动文件
    ├── aloam_velodyne_HDL_32.launch - 32线
    ├── aloam_velodyne_HDL_64.launch - 64线
    ├── aloam_velodyne_VLP_16.launch - 16线
    ├── aloam_velodyne_MY.launch - （原程序没有）适配自己的激光雷达
    ├── kitti_helper.launch - 转换kitti数据集为rosbag
    └── kitti_my.launch - （原程序没有）适配自己的文件路径
├── LICENSE - 授权文件（开源许可证）
├── package.xml - 源代码作者信息和依赖包（库）
├── picture - 原README.md中的图片
    ├── kitti_gif.gif
    └── kitti.png
├── README.md - 本文件
├── rviz_cfg - rviz配置文件
    └── aloam_velodyne.rviz
└── src - 源文件
    ├── kittiHelper.cpp - kitti数据集转换为rosbag的节点
    ├── laserMapping.cpp - 建图节点，根据地图对激光里程计的位姿进行进一步的优化，并将新扫描添加到地图中去
    ├── laserOdometry.cpp - 里程计节点，针对相邻两帧的激光扫描数据进行ICP匹配，估计出两帧之间的位姿变换
    ├── lidarFactor.hpp - 非线性优化因子，包含了里程计节点以及建图节点ICP匹配时非线性优化代价函数中的因子定义
    └── scanRegistration.cpp - 点云注册节点，对激光雷达的数据进行筛选与分类，得到相应的特征点云
```
