<a href="#"><img src="https://img.shields.io/badge/c++-%2300599C.svg?style=flat&logo=c%2B%2B&logoColor=white"></img></a>
  <a href="#"><img src="https://img.shields.io/github/stars/chengwei0427/LIO_Localization"></img></a>
  <a href="#"><img src="https://img.shields.io/github/forks/chengwei0427/LIO_Localization"></img></a>
  <a href="#"><img src="https://img.shields.io/github/repo-size/chengwei0427/LIO_Localization"></img></a>
  <a href="https://github.com/chengwei0427/LIO_Localization/issues"><img src="https://img.shields.io/github/issues/chengwei0427/LIO_Localization"></img></a>
  <a href="https://github.com/chengwei0427/LIO_Localization/graphs/contributors"><img src="https://img.shields.io/github/contributors/chengwei0427/LIO_Localization?color=blue"></img></a>


# LIO_Localization

This repository is a modified LiDAR-inertial odometry system for Spinning LiDAR. The system is developed based on the open-source odometry framework [**LIO-Livox**](https://github.com/Livox-SDK/LIO-Livox) to get the odometry information. And the feature extract moudle is implemented based on [**LIO-SAM**](https://github.com/TixiaoShan/LIO-SAM) .

## Modification

  - Feature extract moudle is implemented based on lio-sam, this moudle support velodyne,ouster and livox lidar;
  - Modify the PoseEstimation moudle , **not rely on sophus**,--[**This part is not uploaded to the repository**].
  - map manager use ikdtree instead of the original two individual (global & local) map；
  


## demo

**Test with west.bag(lio-sam)**
<div align="center">
<img src="./doc/west.png" width="1000px">
</div>


## Prerequisites

*  [Ubuntu](http://ubuntu.com) (tested on 18.04)
*  [ROS](http://wiki.ros.org/ROS/Installation) (tested with Melodic)
*  [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
*  [Ceres Solver](http://ceres-solver.org/installation.html)
*  [PCL](http://www.pointclouds.org/downloads/linux.html)
*  [livox_ros_driver](https://github.com/Livox-SDK/livox_ros_driver)
*  Suitesparse
   ```
   sudo apt-get install libsuitesparse-dev
   ```

## Compilation
```
cd ~/catkin_ws/src
git clone https://github.com/Livox-SDK/LIO-Livox
cd ..
catkin_make
```

## Run with bag files:
### Run the launch file:
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch lio_livox horizon.launch
```

#### Play your bag files:
```
rosbag play YOUR_ROSBAG.bag
```

## Run with your device:
### Run your LiDAR with livox_ros_driver
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch livox_ros_driver livox_lidar_msg.launch
```

### Run the launch file:
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch lio_livox horizon.launch
```

## Notes:

There are some parameters in launch files:
*  IMU_Mode: choose IMU information fusion strategy, there are 3 modes:
    -  0 - without using IMU information, pure LiDAR odometry, motion distortion is removed using a constant velocity model 
    -  1 - using IMU preintegration to remove motion distortion 
    -  2 - tightly coupling IMU and LiDAR information
*  Extrinsic_Tlb: extrinsic parameter between LiDAR and IMU, which uses SE3 form. If you want to use an external IMU, you need to calibrate your own sensor suite
and change this parameter to your extrinsic parameter.

There are also some parameters in the config file:
*  Use_seg: choose the segmentation mode for dynamic objects filtering, there are 2 modes:
    -  0 - without using the segmentation method, you can choose this mode if there is few dynamic objects in your data
    -  1 - using the segmentation method to remove dynamic objects

## Acknowledgements
Thanks for following work:
* [LOAM](https://github.com/cuitaixiang/LOAM_NOTED) (LOAM: Lidar Odometry and Mapping in Real-time)
* [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) (VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator)
* [LIO-mapping](https://github.com/hyye/lio-mapping) (Tightly Coupled 3D Lidar Inertial Odometry and Mapping)
* [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) (ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM)
* [LiLi-OM](https://github.com/KIT-ISAS/lili-om) (Towards High-Performance Solid-State-LiDAR-Inertial Odometry and Mapping)
* [MSCKF_VIO](https://github.com/KumarRobotics/msckf_vio) (Robust Stereo Visual Inertial Odometry for Fast Autonomous Flight)
* [horizon_highway_slam](https://github.com/Livox-SDK/horizon_highway_slam)
* [livox_mapping](https://github.com/Livox-SDK/livox_mapping)
* [livox_horizon_loam](https://github.com/Livox-SDK/livox_horizon_loam)

## Support
