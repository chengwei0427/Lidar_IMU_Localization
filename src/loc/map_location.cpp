#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <deque>
#include <string>
#include <thread>
#include <vector>
#include <array>
#include <chrono>
//  ros
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

//  PCL
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>

//  Eigen
#include <Eigen/Dense>

#include "tool_color_printf.hpp"
#include "mutexDeque.hpp"
#include "tictoc.hpp"
#include "Estimator/Map_Manager.h"
#include "Estimator/ceresfunc.h"

std::string root_dir = ROOT_DIR;

struct PointXYZIRPYT
{
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  float roll;
  float pitch;
  float yaw;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time))

// typedef pcl::PointXYZI PointType;
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> CLOUD;
typedef CLOUD::Ptr CLOUD_PTR;
typedef PointXYZIRPYT PointTypePose;

struct pcdmap
{
  std::vector<CLOUD_PTR> corner_keyframes_;
  std::vector<CLOUD_PTR> surf_keyframes_;
  CLOUD_PTR globalMapCloud_;
  CLOUD_PTR cloudKeyPoses3D_;
  CLOUD_PTR globalCornerMapCloud_;
  CLOUD_PTR globalSurfMapCloud_;
  pcdmap()
  {
    globalMapCloud_.reset(new CLOUD);
    cloudKeyPoses3D_.reset(new CLOUD);
    globalCornerMapCloud_.reset(new CLOUD);
    globalSurfMapCloud_.reset(new CLOUD);
  }
};

struct kylidar
{
  CLOUD_PTR corner;
  CLOUD_PTR surf;
  CLOUD_PTR cloud;
  double time;
  kylidar() : time(-1)
  {
    corner.reset(new CLOUD);
    surf.reset(new CLOUD);
    cloud.reset(new CLOUD());
  }
};

enum InitializedFlag
{
  NonInitialized,
  Initializing,
  Initialized,
  MayLost
};

/** \brief point to line feature */
struct FeatureLine
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d pointOri;
  Eigen::Vector3d lineP1;
  Eigen::Vector3d lineP2;
  double error;
  bool valid;
  FeatureLine(Eigen::Vector3d po, Eigen::Vector3d p1, Eigen::Vector3d p2)
      : pointOri(std::move(po)), lineP1(std::move(p1)), lineP2(std::move(p2))
  {
    valid = false;
    error = 0;
  }
  double ComputeError(const Eigen::Matrix4d &pose)
  {
    Eigen::Vector3d P_to_Map = pose.topLeftCorner(3, 3) * pointOri + pose.topRightCorner(3, 1);
    double l12 = std::sqrt((lineP1(0) - lineP2(0)) * (lineP1(0) - lineP2(0)) + (lineP1(1) - lineP2(1)) * (lineP1(1) - lineP2(1)) + (lineP1(2) - lineP2(2)) * (lineP1(2) - lineP2(2)));
    double a012 = std::sqrt(
        ((P_to_Map(0) - lineP1(0)) * (P_to_Map(1) - lineP2(1)) - (P_to_Map(0) - lineP2(0)) * (P_to_Map(1) - lineP1(1))) * ((P_to_Map(0) - lineP1(0)) * (P_to_Map(1) - lineP2(1)) - (P_to_Map(0) - lineP2(0)) * (P_to_Map(1) - lineP1(1))) + ((P_to_Map(0) - lineP1(0)) * (P_to_Map(2) - lineP2(2)) - (P_to_Map(0) - lineP2(0)) * (P_to_Map(2) - lineP1(2))) * ((P_to_Map(0) - lineP1(0)) * (P_to_Map(2) - lineP2(2)) - (P_to_Map(0) - lineP2(0)) * (P_to_Map(2) - lineP1(2))) + ((P_to_Map(1) - lineP1(1)) * (P_to_Map(2) - lineP2(2)) - (P_to_Map(1) - lineP2(1)) * (P_to_Map(2) - lineP1(2))) * ((P_to_Map(1) - lineP1(1)) * (P_to_Map(2) - lineP2(2)) - (P_to_Map(1) - lineP2(1)) * (P_to_Map(2) - lineP1(2))));
    error = a012 / l12;
  }
};

/** \brief point to plan feature */
struct FeaturePlanVec
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d pointOri;
  Eigen::Vector3d pointProj;
  Eigen::Matrix3d sqrt_info;
  double error;
  bool valid;
  FeaturePlanVec(const Eigen::Vector3d &po, const Eigen::Vector3d &p_proj, Eigen::Matrix3d sqrt_info_)
      : pointOri(po), pointProj(p_proj), sqrt_info(sqrt_info_)
  {
    valid = false;
    error = 0;
  }
  double ComputeError(const Eigen::Matrix4d &pose)
  {
    Eigen::Vector3d P_to_Map = pose.topLeftCorner(3, 3) * pointOri + pose.topRightCorner(3, 1);
    error = (P_to_Map - pointProj).norm();
  }
};

class map_location
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_cloud_;
  ros::Subscriber sub_initial_pose_;

  ros::Publisher pub_corner_map;
  ros::Publisher pub_surf_;
  ros::Publisher pubMappedPoints_;
  ros::Publisher pubLaserOdometryPath_;

  tf::StampedTransform transform_;
  tf::TransformBroadcaster broadcaster_; //  publish laser to map tf

  nav_msgs::Path laserOdoPath;

  pcdmap map;
  std::string filename;
  std::string pointCloudTopic;
  double corner_leaf_;
  double surf_leaf_;

  pcl::KdTreeFLANN<PointType>::Ptr kdtree_keyposes_3d_;
  pcl::KdTreeFLANN<PointType>::Ptr kdtree_corner_map;
  pcl::KdTreeFLANN<PointType>::Ptr kdtree_surf_map;

  pcl::KdTreeFLANN<PointType>::Ptr kdtree_corner_localmap;
  pcl::KdTreeFLANN<PointType>::Ptr kdtree_surf_localmap;

  CLOUD_PTR surround_surf;
  CLOUD_PTR surround_corner;

  pcl::VoxelGrid<PointType> ds_corner_;
  pcl::VoxelGrid<PointType> ds_surf_;

  MutexDeque<kylidar> kframeQueue;
  InitializedFlag initializedFlag;

  PointTypePose initpose;
  Eigen::Matrix4d pose_in_map;

  MAP_MANAGER *map_manager;
  static const int SLIDEWINDOWSIZE = 2;
  double para_PR[SLIDEWINDOWSIZE][6];
  double para_VBias[SLIDEWINDOWSIZE][9];
  Eigen::Matrix4d exTlb = Eigen::Matrix4d::Identity();
  double thres_dist = 1.0;
  double plan_weight_tan = 0.0;
  Eigen::Matrix3d delta_Rl = Eigen::Matrix3d::Identity();
  Eigen::Vector3d delta_tl = Eigen::Vector3d::Zero();
  Eigen::Matrix4d transformLastMapped = Eigen::Matrix4d::Identity();

  static const int localMapWindowSize = 30;
  int localMapID = 0;
  pcl::PointCloud<PointType>::Ptr localCornerMap[localMapWindowSize];
  pcl::PointCloud<PointType>::Ptr localSurfMap[localMapWindowSize];
  pcl::PointCloud<PointType>::Ptr laserCloudCornerFromLocal;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfFromLocal;

public:
  map_location()
  {
    nh_.param<std::string>("location/filedir", filename, "");
    nh_.param<std::string>("location/pointCloudTopic", pointCloudTopic, "points_raw");
    nh_.param<double>("location/corner_leaf_", corner_leaf_, 0.2);
    nh_.param<double>("location/surf_leaf_", surf_leaf_, 0.5);

    sub_cloud_ = nh_.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 50, &map_location::cloudHandler, this);
    sub_initial_pose_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, &map_location::initialPoseCB, this);

    pub_corner_map = nh_.advertise<sensor_msgs::PointCloud2>("/global_corner_map", 1);
    pub_surf_ = nh_.advertise<sensor_msgs::PointCloud2>("/surf_registed", 1);

    pubMappedPoints_ = nh_.advertise<sensor_msgs::PointCloud2>("/laser_cloud_mapped", 10);
    pubLaserOdometryPath_ = nh_.advertise<nav_msgs::Path>("/path_mapped", 5);

    ds_corner_.setLeafSize(corner_leaf_, corner_leaf_, corner_leaf_);
    ds_surf_.setLeafSize(surf_leaf_, surf_leaf_, surf_leaf_);

    if (loadmap())
      std::cout << ANSI_COLOR_GREEN << "load map successful..." << ANSI_COLOR_RESET << std::endl;
    else
    {
      std::cout << ANSI_COLOR_RED_BOLD << "WARN: load map failed." << ANSI_COLOR_RESET << std::endl;
      return;
    }

    surround_surf.reset(new CLOUD);
    surround_corner.reset(new CLOUD);
    kdtree_keyposes_3d_.reset(new pcl::KdTreeFLANN<PointType>());
    kdtree_keyposes_3d_->setInputCloud(map.cloudKeyPoses3D_); // init 3d-pose kdtree

    kdtree_corner_map.reset(new pcl::KdTreeFLANN<PointType>());
    kdtree_corner_map->setInputCloud(map.globalCornerMapCloud_);
    kdtree_surf_map.reset(new pcl::KdTreeFLANN<PointType>());
    kdtree_surf_map->setInputCloud(map.globalSurfMapCloud_);

    kdtree_corner_localmap.reset(new pcl::KdTreeFLANN<PointType>());
    kdtree_surf_localmap.reset(new pcl::KdTreeFLANN<PointType>());

    if (pub_corner_map.getNumSubscribers() > 0)
    {
      sensor_msgs::PointCloud2 msg_corner_target;
      pcl::toROSMsg(*map.globalCornerMapCloud_, msg_corner_target);
      msg_corner_target.header.stamp = ros::Time::now();
      msg_corner_target.header.frame_id = "world";
      pub_corner_map.publish(msg_corner_target);
      std::cout << "publish corner map,size: " << map.globalCornerMapCloud_->size() << std::endl;
    }
    initializedFlag = NonInitialized;

    for (int i = 0; i < localMapWindowSize; i++)
    {
      localCornerMap[i].reset(new pcl::PointCloud<PointType>);
      localSurfMap[i].reset(new pcl::PointCloud<PointType>);
    }
    laserCloudCornerFromLocal.reset(new pcl::PointCloud<PointType>);
    laserCloudSurfFromLocal.reset(new pcl::PointCloud<PointType>);

    pose_in_map = Eigen::Matrix4d::Identity();
    map_manager = new MAP_MANAGER(0.2, 0.3);
  }
  ~map_location();

  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
  {
    kylidar kframe;
    pcl::fromROSMsg(*laserCloudMsg, *kframe.cloud);
    kframe.time = laserCloudMsg->header.stamp.toSec();

    if (!kframeQueue.empty() && (initializedFlag != Initialized)) //  由于tf关系，导致发布时间存在滞后，tf无法显示
      kframeQueue.clear();
    kframeQueue.push_back(kframe);
  }

  void ExtractFeature(kylidar &kf)
  {
    for (const auto &p : kf.cloud->points)
    {
      if (std::fabs(p.normal_z - 1.0) < 1e-5)
        kf.corner->push_back(p);
    }
    for (const auto &p : kf.cloud->points)
    {
      if (std::fabs(p.normal_z - 2.0) < 1e-5)
        kf.surf->push_back(p);
    }
    ds_surf_.setInputCloud(kf.surf);
    ds_surf_.filter(*kf.surf);
    ds_corner_.setInputCloud(kf.corner);
    ds_corner_.filter(*kf.corner);
  }

  void initialPoseCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
  {
    //  低频，不加锁
    PointType p;

    initpose.x = msg->pose.pose.position.x;
    initpose.y = msg->pose.pose.position.y;
    initpose.z = msg->pose.pose.position.z;
    double roll, pitch, yaw;
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    initpose.roll = roll;
    initpose.pitch = pitch;
    initpose.yaw = yaw;

    p.x = initpose.x;
    p.y = initpose.y;
    p.z = initpose.z;
    std::cout << ANSI_COLOR_RED << "Get initial pose: " << initpose.x << " " << initpose.y << " " << initpose.z << ANSI_COLOR_RESET << std::endl;
    extractSurroundKeyFrames(p);
    std::cout << ANSI_COLOR_YELLOW << "Change flat from " << initializedFlag
              << " to " << Initializing << ", start do localizating ..."
              << ANSI_COLOR_RESET << std::endl;
    if (initializedFlag != NonInitialized)
    { // TODO: 非第一次执行，需要重置部分参数
      delta_Rl = Eigen::Matrix3d::Identity();
      delta_tl = Eigen::Vector3d::Zero();
    }
    initializedFlag = Initializing;
  }

  void pubOdometry(Eigen::Matrix4d &pose, double &time)
  {
    Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
    ros::Time ros_time = ros::Time().fromSec(time);
    transform_.stamp_ = ros_time;
    transform_.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    transform_.setOrigin(tf::Vector3(pose(0, 3), pose(1, 3), pose(2, 3)));
    transform_.frame_id_ = "world";
    transform_.child_frame_id_ = "base_link";
    broadcaster_.sendTransform(transform_);

    geometry_msgs::PoseStamped laserPose;
    laserPose.header.frame_id = "world";
    laserPose.header.stamp = ros_time;

    laserPose.pose.orientation.x = q.x();
    laserPose.pose.orientation.y = q.y();
    laserPose.pose.orientation.z = q.z();
    laserPose.pose.orientation.w = q.w();
    laserPose.pose.position.x = pose(0, 3);
    laserPose.pose.position.y = pose(1, 3);
    laserPose.pose.position.z = pose(2, 3);

    laserOdoPath.header.stamp = ros_time;
    laserOdoPath.poses.push_back(laserPose);
    laserOdoPath.header.frame_id = "/world";
    pubLaserOdometryPath_.publish(laserOdoPath);
  }

  void Estimate(kylidar &frame)
  {
    int num_corner_map = 0;
    int num_surf_map = 0;
    int windowSize = 1;

    kdtree_corner_localmap->setInputCloud(laserCloudCornerFromLocal);
    kdtree_surf_localmap->setInputCloud(laserCloudSurfFromLocal);

    // store point to line features
    std::vector<std::vector<FeatureLine>>
        vLineFeatures(windowSize);
    for (auto &v : vLineFeatures)
    {
      v.reserve(2000);
    }
    // store point to plan features
    std::vector<std::vector<FeaturePlanVec>> vPlanFeatures(windowSize);
    for (auto &v : vPlanFeatures)
    {
      v.reserve(2000);
    }
    // excute optimize process
    const int max_iters = 5;
    int iterOpt = 0;
    for (; iterOpt < max_iters; ++iterOpt)
    {
      {
        Eigen::Map<Eigen::Matrix<double, 6, 1>> PR(para_PR[0]);
        PR.segment<3>(0) = Eigen::Vector3d(pose_in_map(0, 3), pose_in_map(1, 3), pose_in_map(2, 3));
        Eigen::Quaterniond Q(pose_in_map.block<3, 3>(0, 0));
        Q.normalized();
        PR.segment<3>(3) = Sophus::SO3d(Q).log();

        Eigen::Map<Eigen::Matrix<double, 9, 1>> VBias(para_VBias[0]);
        VBias.segment<3>(0) = Eigen::Vector3d(0, 0, 0);
        VBias.segment<3>(3) = Eigen::Vector3d(0, 0, 0);
        VBias.segment<3>(6) = Eigen::Vector3d(0, 0, 0);
      }

      // create huber loss function
      ceres::LossFunction *loss_function = NULL;
      loss_function = new ceres::HuberLoss(300);

      ceres::Problem::Options problem_options;
      ceres::Problem problem(problem_options);

      for (int i = 0; i < windowSize; ++i)
      {
        problem.AddParameterBlock(para_PR[i], 6);
      }

      for (int i = 0; i < windowSize; ++i)
        problem.AddParameterBlock(para_VBias[i], 9);

      Eigen::Quaterniond q_before_opti(pose_in_map.block<3, 3>(0, 0));
      Eigen::Vector3d t_before_opti = Eigen::Vector3d(pose_in_map(0, 3), pose_in_map(1, 3), pose_in_map(2, 3));

      std::vector<std::vector<ceres::CostFunction *>> edgesLine(windowSize);
      std::vector<std::vector<ceres::CostFunction *>> edgesPlan(windowSize);
      std::thread threads[2];
      for (int f = 0; f < windowSize; ++f)
      {
        Eigen::Matrix4d transformTobeMapped = Eigen::Matrix4d::Identity();
        transformTobeMapped = pose_in_map;

        threads[0] = std::thread(&map_location::processPointToLine, this,
                                 std::ref(edgesLine[f]),
                                 std::ref(vLineFeatures[f]),
                                 std::ref(frame.corner),
                                 std::ref(map.globalCornerMapCloud_),
                                 std::ref(kdtree_corner_map),
                                 std::ref(laserCloudCornerFromLocal),
                                 std::ref(kdtree_corner_localmap),
                                 std::ref(exTlb),
                                 std::ref(transformTobeMapped));

        threads[1] = std::thread(&map_location::processPointToPlanVec, this,
                                 std::ref(edgesPlan[f]),
                                 std::ref(vPlanFeatures[f]),
                                 std::ref(frame.surf),
                                 std::ref(map.globalSurfMapCloud_),
                                 std::ref(kdtree_surf_map),
                                 std::ref(laserCloudSurfFromLocal),
                                 std::ref(kdtree_surf_localmap),
                                 std::ref(exTlb),
                                 std::ref(transformTobeMapped));

        threads[0].join();
        threads[1].join();
      }

      if (windowSize == SLIDEWINDOWSIZE)
      {
        thres_dist = 1.0;
        if (iterOpt == 0)
        {
          for (int f = 0; f < windowSize; ++f)
          {
            int cntFtu = 0;
            for (auto &e : edgesLine[f])
            {
              if (std::fabs(vLineFeatures[f][cntFtu].error) > 1e-5)
              {
                problem.AddResidualBlock(e, loss_function, para_PR[f]);
                vLineFeatures[f][cntFtu].valid = true;
              }
              else
              {
                vLineFeatures[f][cntFtu].valid = false;
              }
              cntFtu++;
            }

            cntFtu = 0;
            for (auto &e : edgesPlan[f])
            {
              if (std::fabs(vPlanFeatures[f][cntFtu].error) > 1e-5)
              {
                problem.AddResidualBlock(e, loss_function, para_PR[f]);
                vPlanFeatures[f][cntFtu].valid = true;
              }
              else
              {
                vPlanFeatures[f][cntFtu].valid = false;
              }
              cntFtu++;
            }
          }
        }
        else
        {
          for (int f = 0; f < windowSize; ++f)
          {
            int cntFtu = 0;
            for (auto &e : edgesLine[f])
            {
              if (vLineFeatures[f][cntFtu].valid)
              {
                problem.AddResidualBlock(e, loss_function, para_PR[f]);
              }
              cntFtu++;
            }
            cntFtu = 0;
            for (auto &e : edgesPlan[f])
            {
              if (vPlanFeatures[f][cntFtu].valid)
              {
                problem.AddResidualBlock(e, loss_function, para_PR[f]);
              }
              cntFtu++;
            }
          }
        }
      }
      else
      {
        if (iterOpt == 0)
        {
          thres_dist = 10.0;
        }
        else
        {
          thres_dist = 1.0;
        }
        for (int f = 0; f < windowSize; ++f)
        {
          int cntFtu = 0;
          for (auto &e : edgesLine[f])
          {
            if (std::fabs(vLineFeatures[f][cntFtu].error) > 1e-5)
            {
              problem.AddResidualBlock(e, loss_function, para_PR[f]);
              vLineFeatures[f][cntFtu].valid = true;
            }
            else
            {
              vLineFeatures[f][cntFtu].valid = false;
            }
            cntFtu++;
          }
          cntFtu = 0;
          for (auto &e : edgesPlan[f])
          {
            if (std::fabs(vPlanFeatures[f][cntFtu].error) > 1e-5)
            {
              problem.AddResidualBlock(e, loss_function, para_PR[f]);
              vPlanFeatures[f][cntFtu].valid = true;
            }
            else
            {
              vPlanFeatures[f][cntFtu].valid = false;
            }
            cntFtu++;
          }
        }
      }

      ceres::Solver::Options options;
      options.linear_solver_type = ceres::DENSE_SCHUR;
      options.trust_region_strategy_type = ceres::DOGLEG;
      options.max_num_iterations = 10;
      options.minimizer_progress_to_stdout = false;
      options.num_threads = 6;
      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);

      {
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> PR(para_PR[0]);
        Eigen::Map<const Eigen::Matrix<double, 9, 1>> VBias(para_VBias[0]);
        pose_in_map.block<3, 3>(0, 0) = Sophus::SO3d::exp(PR.segment<3>(3)).unit_quaternion().matrix();
        pose_in_map.block<3, 1>(0, 3) = PR.segment<3>(0);
      }

      Eigen::Quaterniond q_after_opti(pose_in_map.block<3, 3>(0, 0));
      Eigen::Vector3d t_after_opti(pose_in_map.block<3, 1>(0, 3));

      double deltaR = (q_before_opti.angularDistance(q_after_opti)) * 180.0 / M_PI;
      double deltaT = (t_before_opti - t_after_opti).norm();

      if (deltaR < 0.05 && deltaT < 0.05 || (iterOpt + 1) == max_iters)
      {
        if (windowSize != SLIDEWINDOWSIZE)
          break;
      }
      if (windowSize != SLIDEWINDOWSIZE)
      {
        for (int f = 0; f < windowSize; ++f)
        {
          edgesLine[f].clear();
          edgesPlan[f].clear();
          vLineFeatures[f].clear();
          vPlanFeatures[f].clear();
        }
      }
    }
    std::cout << "estimate iter: " << iterOpt << std::endl;
  }

  void run()
  {
    while (true)
    {

      if (initializedFlag == NonInitialized)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      if (kframeQueue.empty())
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      kylidar kframe = kframeQueue.pop_front();
      RemoveLidarDistortion(kframe, delta_Rl, delta_tl);

      sensor_msgs::PointCloud2 laserCloudMsg;
      pcl::toROSMsg(*kframe.cloud, laserCloudMsg);
      laserCloudMsg.header.frame_id = "/base_link";
      laserCloudMsg.header.stamp.fromSec(kframe.time);
      pubMappedPoints_.publish(laserCloudMsg);

      if (initializedFlag == Initializing)
      {
        if (ICPScanMatchGlobal(kframe))
        {
          initializedFlag = Initialized;
          std::cout << ANSI_COLOR_GREEN << "icp scan match successful ..." << ANSI_COLOR_RESET << std::endl;
        }
        pubOdometry(pose_in_map, kframe.time);
      }
      else if (initializedFlag == Initialized)
      {
        TicToc tc;
        double t1, t2, t3;
        //  TODO: 增加局部地图
        int laserCloudCornerFromLocalNum = laserCloudCornerFromLocal->points.size();
        int laserCloudSurfFromLocalNum = laserCloudSurfFromLocal->points.size();
        if ((kdtree_surf_map && kdtree_corner_map) ||
            (laserCloudCornerFromLocalNum > 0 && laserCloudSurfFromLocalNum > 100))
        {
          pose_in_map.topRightCorner(3, 1) = transformLastMapped.topLeftCorner(3, 3) * delta_tl + transformLastMapped.topRightCorner(3, 1);
          pose_in_map.topLeftCorner(3, 3) = transformLastMapped.topLeftCorner(3, 3) * delta_Rl;
          tc.tic();
          Estimate(kframe);
          t1 = tc.toc();
          tc.tic();
          pubOdometry(pose_in_map, kframe.time);
          Eigen::Matrix4d transformAftMapped = Eigen::Matrix4d::Identity();
          transformAftMapped = pose_in_map;
          Eigen::Matrix3d Rwlpre = transformLastMapped.topLeftCorner(3, 3);
          Eigen::Vector3d Pwlpre = transformLastMapped.topRightCorner(3, 1);
          delta_Rl = Rwlpre.transpose() * transformAftMapped.topLeftCorner(3, 3);
          delta_tl = Rwlpre.transpose() * (transformAftMapped.topRightCorner(3, 1) - Pwlpre);
          transformLastMapped = pose_in_map;
          t2 = tc.toc();
        }
        tc.tic();
        MapIncrementLocal(kframe);
        t3 = tc.toc();
        std::cout << "LIO takes: " << t1 << ",pubodom:" << t2 << ",mapincrement:" << t3 << std::endl;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  void RemoveLidarDistortion(kylidar &kf,
                             const Eigen::Matrix3d &dRlc, const Eigen::Vector3d &dtlc)
  {
    int PointsNum = kf.cloud->points.size();
    for (int i = 0; i < PointsNum; i++)
    {
      Eigen::Vector3d startP;
      float s = kf.cloud->points[i].normal_x; //  time intervel
      Eigen::Quaterniond qlc = Eigen::Quaterniond(dRlc).normalized();
      Eigen::Quaterniond delta_qlc = Eigen::Quaterniond::Identity().slerp(s, qlc).normalized(); // 插值
      const Eigen::Vector3d delta_Plc = s * dtlc;
      startP = delta_qlc * Eigen::Vector3d(kf.cloud->points[i].x, kf.cloud->points[i].y, kf.cloud->points[i].z) + delta_Plc;
      Eigen::Vector3d _po = dRlc.transpose() * (startP - dtlc);

      kf.cloud->points[i].x = _po(0);
      kf.cloud->points[i].y = _po(1);
      kf.cloud->points[i].z = _po(2);
      kf.cloud->points[i].normal_x = 1.0;

      if (std::fabs(kf.cloud->points[i].normal_z - 1.0) < 1e-5)
        kf.corner->push_back(kf.cloud->points[i]);
      if (std::fabs(kf.cloud->points[i].normal_z - 2.0) < 1e-5)
        kf.surf->push_back(kf.cloud->points[i]);
    }
    std::cout << "bef-surf: " << kf.surf->size() << ",corner: " << kf.corner->size() << std::endl;
    ds_surf_.setInputCloud(kf.surf);
    ds_surf_.filter(*kf.surf);
    ds_corner_.setInputCloud(kf.corner);
    ds_corner_.filter(*kf.corner);
    std::cout << "aft-surf: " << kf.surf->size() << ",corner: " << kf.corner->size() << std::endl;
  }

  void processPointToLine(std::vector<ceres::CostFunction *> &edges,
                          std::vector<FeatureLine> &vLineFeatures,
                          const pcl::PointCloud<PointType>::Ptr &laserCloudCorner,
                          const pcl::PointCloud<PointType>::Ptr &laserCloudCornerGlobal,
                          const pcl::KdTreeFLANN<PointType>::Ptr &kdtreeGlobal,
                          const pcl::PointCloud<PointType>::Ptr &laserCloudCornerLocal,
                          const pcl::KdTreeFLANN<PointType>::Ptr &kdtreeLocal,
                          const Eigen::Matrix4d &exTlb,
                          const Eigen::Matrix4d &m4d)
  {

    Eigen::Matrix4d Tbl = Eigen::Matrix4d::Identity();
    Tbl.topLeftCorner(3, 3) = exTlb.topLeftCorner(3, 3).transpose();
    Tbl.topRightCorner(3, 1) = -1.0 * Tbl.topLeftCorner(3, 3) * exTlb.topRightCorner(3, 1);
    if (!vLineFeatures.empty())
    {
      for (const auto &l : vLineFeatures)
      {
        auto *e = Cost_NavState_IMU_Line::Create(l.pointOri,
                                                 l.lineP1,
                                                 l.lineP2,
                                                 Tbl,
                                                 Eigen::Matrix<double, 1, 1>(1 / IMUIntegrator::lidar_m));
        edges.push_back(e);
      }
      return;
    }
    PointType _pointOri, _pointSel, _coeff;
    std::vector<int> _pointSearchInd;
    std::vector<float> _pointSearchSqDis;
    std::vector<int> _pointSearchInd2;
    std::vector<float> _pointSearchSqDis2;

    Eigen::Matrix<double, 3, 3> _matA1;
    _matA1.setZero();

    int laserCloudCornerStackNum = laserCloudCorner->points.size();
    pcl::PointCloud<PointType>::Ptr kd_pointcloud(new pcl::PointCloud<PointType>);
    int debug_num1 = 0;
    int debug_num2 = 0;
    int debug_num12 = 0;
    int debug_num22 = 0;
    for (int i = 0; i < laserCloudCornerStackNum; i++)
    {
      _pointOri = laserCloudCorner->points[i];
      MAP_MANAGER::pointAssociateToMap(&_pointOri, &_pointSel, m4d);

      //  for global
      if (laserCloudCornerGlobal->points.size() > 100)
      {
        kdtreeGlobal->nearestKSearch(_pointSel, 5, _pointSearchInd2, _pointSearchSqDis2);
        if (_pointSearchSqDis2[4] < thres_dist)
        {

          debug_num2++;
          float cx = 0;
          float cy = 0;
          float cz = 0;
          for (int j = 0; j < 5; j++)
          {
            cx += laserCloudCornerGlobal->points[_pointSearchInd2[j]].x;
            cy += laserCloudCornerGlobal->points[_pointSearchInd2[j]].y;
            cz += laserCloudCornerGlobal->points[_pointSearchInd2[j]].z;
          }
          cx /= 5;
          cy /= 5;
          cz /= 5;

          float a11 = 0;
          float a12 = 0;
          float a13 = 0;
          float a22 = 0;
          float a23 = 0;
          float a33 = 0;
          for (int j = 0; j < 5; j++)
          {
            float ax = laserCloudCornerGlobal->points[_pointSearchInd2[j]].x - cx;
            float ay = laserCloudCornerGlobal->points[_pointSearchInd2[j]].y - cy;
            float az = laserCloudCornerGlobal->points[_pointSearchInd2[j]].z - cz;

            a11 += ax * ax;
            a12 += ax * ay;
            a13 += ax * az;
            a22 += ay * ay;
            a23 += ay * az;
            a33 += az * az;
          }
          a11 /= 5;
          a12 /= 5;
          a13 /= 5;
          a22 /= 5;
          a23 /= 5;
          a33 /= 5;

          _matA1(0, 0) = a11;
          _matA1(0, 1) = a12;
          _matA1(0, 2) = a13;
          _matA1(1, 0) = a12;
          _matA1(1, 1) = a22;
          _matA1(1, 2) = a23;
          _matA1(2, 0) = a13;
          _matA1(2, 1) = a23;
          _matA1(2, 2) = a33;

          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(_matA1);
          Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);

          if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
          {
            debug_num22++;
            float x1 = cx + 0.1 * unit_direction[0];
            float y1 = cy + 0.1 * unit_direction[1];
            float z1 = cz + 0.1 * unit_direction[2];
            float x2 = cx - 0.1 * unit_direction[0];
            float y2 = cy - 0.1 * unit_direction[1];
            float z2 = cz - 0.1 * unit_direction[2];

            Eigen::Vector3d tripod1(x1, y1, z1);
            Eigen::Vector3d tripod2(x2, y2, z2);
            auto *e = Cost_NavState_IMU_Line::Create(Eigen::Vector3d(_pointOri.x, _pointOri.y, _pointOri.z),
                                                     tripod1,
                                                     tripod2,
                                                     Tbl,
                                                     Eigen::Matrix<double, 1, 1>(1 / IMUIntegrator::lidar_m));
            edges.push_back(e);
            vLineFeatures.emplace_back(Eigen::Vector3d(_pointOri.x, _pointOri.y, _pointOri.z),
                                       tripod1,
                                       tripod2);
            vLineFeatures.back().ComputeError(m4d);
          }
        }
      }

      if (laserCloudCornerLocal->points.size() > 20)
      {
        kdtreeLocal->nearestKSearch(_pointSel, 5, _pointSearchInd2, _pointSearchSqDis2);
        if (_pointSearchSqDis2[4] < thres_dist)
        {

          debug_num2++;
          float cx = 0;
          float cy = 0;
          float cz = 0;
          for (int j = 0; j < 5; j++)
          {
            cx += laserCloudCornerLocal->points[_pointSearchInd2[j]].x;
            cy += laserCloudCornerLocal->points[_pointSearchInd2[j]].y;
            cz += laserCloudCornerLocal->points[_pointSearchInd2[j]].z;
          }
          cx /= 5;
          cy /= 5;
          cz /= 5;

          float a11 = 0;
          float a12 = 0;
          float a13 = 0;
          float a22 = 0;
          float a23 = 0;
          float a33 = 0;
          for (int j = 0; j < 5; j++)
          {
            float ax = laserCloudCornerLocal->points[_pointSearchInd2[j]].x - cx;
            float ay = laserCloudCornerLocal->points[_pointSearchInd2[j]].y - cy;
            float az = laserCloudCornerLocal->points[_pointSearchInd2[j]].z - cz;

            a11 += ax * ax;
            a12 += ax * ay;
            a13 += ax * az;
            a22 += ay * ay;
            a23 += ay * az;
            a33 += az * az;
          }
          a11 /= 5;
          a12 /= 5;
          a13 /= 5;
          a22 /= 5;
          a23 /= 5;
          a33 /= 5;

          _matA1(0, 0) = a11;
          _matA1(0, 1) = a12;
          _matA1(0, 2) = a13;
          _matA1(1, 0) = a12;
          _matA1(1, 1) = a22;
          _matA1(1, 2) = a23;
          _matA1(2, 0) = a13;
          _matA1(2, 1) = a23;
          _matA1(2, 2) = a33;

          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(_matA1);
          Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);

          if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
          {
            debug_num22++;
            float x1 = cx + 0.1 * unit_direction[0];
            float y1 = cy + 0.1 * unit_direction[1];
            float z1 = cz + 0.1 * unit_direction[2];
            float x2 = cx - 0.1 * unit_direction[0];
            float y2 = cy - 0.1 * unit_direction[1];
            float z2 = cz - 0.1 * unit_direction[2];

            Eigen::Vector3d tripod1(x1, y1, z1);
            Eigen::Vector3d tripod2(x2, y2, z2);
            auto *e = Cost_NavState_IMU_Line::Create(Eigen::Vector3d(_pointOri.x, _pointOri.y, _pointOri.z),
                                                     tripod1,
                                                     tripod2,
                                                     Tbl,
                                                     Eigen::Matrix<double, 1, 1>(1 / IMUIntegrator::lidar_m));
            edges.push_back(e);
            vLineFeatures.emplace_back(Eigen::Vector3d(_pointOri.x, _pointOri.y, _pointOri.z),
                                       tripod1,
                                       tripod2);
            vLineFeatures.back().ComputeError(m4d);
          }
        }
      }
    }
  }

  void processPointToPlanVec(std::vector<ceres::CostFunction *> &edges,
                             std::vector<FeaturePlanVec> &vPlanFeatures,
                             const pcl::PointCloud<PointType>::Ptr &laserCloudSurf,
                             const pcl::PointCloud<PointType>::Ptr &laserCloudSurfGlobal,
                             const pcl::KdTreeFLANN<PointType>::Ptr &kdtreeGlobal,
                             const pcl::PointCloud<PointType>::Ptr &laserCloudSurfLocal,
                             const pcl::KdTreeFLANN<PointType>::Ptr &kdtreeLocal,
                             const Eigen::Matrix4d &exTlb,
                             const Eigen::Matrix4d &m4d)
  {
    Eigen::Matrix4d Tbl = Eigen::Matrix4d::Identity();
    Tbl.topLeftCorner(3, 3) = exTlb.topLeftCorner(3, 3).transpose();
    Tbl.topRightCorner(3, 1) = -1.0 * Tbl.topLeftCorner(3, 3) * exTlb.topRightCorner(3, 1);
    if (!vPlanFeatures.empty())
    {
      for (const auto &p : vPlanFeatures)
      {
        auto *e = Cost_NavState_IMU_Plan_Vec::Create(p.pointOri,
                                                     p.pointProj,
                                                     Tbl,
                                                     p.sqrt_info);
        edges.push_back(e);
      }
      return;
    }
    PointType _pointOri, _pointSel, _coeff;
    std::vector<int> _pointSearchInd;
    std::vector<float> _pointSearchSqDis;
    std::vector<int> _pointSearchInd2;
    std::vector<float> _pointSearchSqDis2;

    Eigen::Matrix<double, 5, 3> _matA0;
    _matA0.setZero();
    Eigen::Matrix<double, 5, 1> _matB0;
    _matB0.setOnes();
    _matB0 *= -1;
    Eigen::Matrix<double, 3, 1> _matX0;
    _matX0.setZero();
    int laserCloudSurfStackNum = laserCloudSurf->points.size();

    int debug_num1 = 0;
    int debug_num2 = 0;
    int debug_num12 = 0;
    int debug_num22 = 0;
    for (int i = 0; i < laserCloudSurfStackNum; i++)
    {
      _pointOri = laserCloudSurf->points[i];
      MAP_MANAGER::pointAssociateToMap(&_pointOri, &_pointSel, m4d);
      //  for global
      if (laserCloudSurfGlobal->points.size() > 200)
      {
        kdtreeGlobal->nearestKSearch(_pointSel, 5, _pointSearchInd2, _pointSearchSqDis2);
        if (_pointSearchSqDis2[4] < thres_dist)
        {
          debug_num2++;
          for (int j = 0; j < 5; j++)
          {
            _matA0(j, 0) = laserCloudSurfGlobal->points[_pointSearchInd2[j]].x;
            _matA0(j, 1) = laserCloudSurfGlobal->points[_pointSearchInd2[j]].y;
            _matA0(j, 2) = laserCloudSurfGlobal->points[_pointSearchInd2[j]].z;
          }
          _matX0 = _matA0.colPivHouseholderQr().solve(_matB0);

          float pa = _matX0(0, 0);
          float pb = _matX0(1, 0);
          float pc = _matX0(2, 0);
          float pd = 1;

          float ps = std::sqrt(pa * pa + pb * pb + pc * pc);
          pa /= ps;
          pb /= ps;
          pc /= ps;
          pd /= ps;

          bool planeValid = true;
          for (int j = 0; j < 5; j++)
          {
            if (std::fabs(pa * laserCloudSurfGlobal->points[_pointSearchInd2[j]].x +
                          pb * laserCloudSurfGlobal->points[_pointSearchInd2[j]].y +
                          pc * laserCloudSurfGlobal->points[_pointSearchInd2[j]].z + pd) > 0.2)
            {
              planeValid = false;
              break;
            }
          }

          if (planeValid)
          {
            debug_num22++;
            double dist = pa * _pointSel.x +
                          pb * _pointSel.y +
                          pc * _pointSel.z + pd;
            Eigen::Vector3d omega(pa, pb, pc);
            Eigen::Vector3d point_proj = Eigen::Vector3d(_pointSel.x, _pointSel.y, _pointSel.z) - (dist * omega);
            Eigen::Vector3d e1(1, 0, 0);
            Eigen::Matrix3d J = e1 * omega.transpose();
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
            Eigen::Matrix3d R_svd = svd.matrixV() * svd.matrixU().transpose();
            Eigen::Matrix3d info = (1.0 / IMUIntegrator::lidar_m) * Eigen::Matrix3d::Identity();
            info(1, 1) *= plan_weight_tan;
            info(2, 2) *= plan_weight_tan;
            Eigen::Matrix3d sqrt_info = info * R_svd.transpose();

            auto *e = Cost_NavState_IMU_Plan_Vec::Create(Eigen::Vector3d(_pointOri.x, _pointOri.y, _pointOri.z),
                                                         point_proj,
                                                         Tbl,
                                                         sqrt_info);
            edges.push_back(e);
            vPlanFeatures.emplace_back(Eigen::Vector3d(_pointOri.x, _pointOri.y, _pointOri.z),
                                       point_proj,
                                       sqrt_info);
            vPlanFeatures.back().ComputeError(m4d);
          }
        }
      }

      if (laserCloudSurfLocal->points.size() > 20)
      {
        kdtreeLocal->nearestKSearch(_pointSel, 5, _pointSearchInd2, _pointSearchSqDis2);
        if (_pointSearchSqDis2[4] < thres_dist)
        {
          debug_num2++;
          for (int j = 0; j < 5; j++)
          {
            _matA0(j, 0) = laserCloudSurfLocal->points[_pointSearchInd2[j]].x;
            _matA0(j, 1) = laserCloudSurfLocal->points[_pointSearchInd2[j]].y;
            _matA0(j, 2) = laserCloudSurfLocal->points[_pointSearchInd2[j]].z;
          }
          _matX0 = _matA0.colPivHouseholderQr().solve(_matB0);

          float pa = _matX0(0, 0);
          float pb = _matX0(1, 0);
          float pc = _matX0(2, 0);
          float pd = 1;

          float ps = std::sqrt(pa * pa + pb * pb + pc * pc);
          pa /= ps;
          pb /= ps;
          pc /= ps;
          pd /= ps;

          bool planeValid = true;
          for (int j = 0; j < 5; j++)
          {
            if (std::fabs(pa * laserCloudSurfLocal->points[_pointSearchInd2[j]].x +
                          pb * laserCloudSurfLocal->points[_pointSearchInd2[j]].y +
                          pc * laserCloudSurfLocal->points[_pointSearchInd2[j]].z + pd) > 0.2)
            {
              planeValid = false;
              break;
            }
          }

          if (planeValid)
          {
            debug_num22++;
            double dist = pa * _pointSel.x +
                          pb * _pointSel.y +
                          pc * _pointSel.z + pd;
            Eigen::Vector3d omega(pa, pb, pc);
            Eigen::Vector3d point_proj = Eigen::Vector3d(_pointSel.x, _pointSel.y, _pointSel.z) - (dist * omega);
            Eigen::Vector3d e1(1, 0, 0);
            Eigen::Matrix3d J = e1 * omega.transpose();
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
            Eigen::Matrix3d R_svd = svd.matrixV() * svd.matrixU().transpose();
            Eigen::Matrix3d info = (1.0 / IMUIntegrator::lidar_m) * Eigen::Matrix3d::Identity();
            info(1, 1) *= plan_weight_tan;
            info(2, 2) *= plan_weight_tan;
            Eigen::Matrix3d sqrt_info = info * R_svd.transpose();

            auto *e = Cost_NavState_IMU_Plan_Vec::Create(Eigen::Vector3d(_pointOri.x, _pointOri.y, _pointOri.z),
                                                         point_proj,
                                                         Tbl,
                                                         sqrt_info);
            edges.push_back(e);
            vPlanFeatures.emplace_back(Eigen::Vector3d(_pointOri.x, _pointOri.y, _pointOri.z),
                                       point_proj,
                                       sqrt_info);
            vPlanFeatures.back().ComputeError(m4d);
          }
        }
      }
    }
  }

  void MapIncrementLocal(kylidar &kframe)
  {
    int laserCloudCornerStackNum = kframe.corner->points.size();
    int laserCloudSurfStackNum = kframe.surf->points.size();
    PointType pointSel;
    PointType pointSel2;
    size_t Id = localMapID % localMapWindowSize;
    localCornerMap[Id]->clear();
    localSurfMap[Id]->clear();
    for (int i = 0; i < laserCloudCornerStackNum; i++)
    {
      MAP_MANAGER::pointAssociateToMap(&kframe.corner->points[i], &pointSel, pose_in_map);
      localCornerMap[Id]->push_back(pointSel);
    }
    for (int i = 0; i < laserCloudSurfStackNum; i++)
    {
      MAP_MANAGER::pointAssociateToMap(&kframe.surf->points[i], &pointSel2, pose_in_map);
      localSurfMap[Id]->push_back(pointSel2);
    }

    for (int i = 0; i < localMapWindowSize; i++)
    {
      *laserCloudCornerFromLocal += *localCornerMap[i];
      *laserCloudSurfFromLocal += *localSurfMap[i];
    }
    pcl::PointCloud<PointType>::Ptr temp(new pcl::PointCloud<PointType>());
    ds_corner_.setInputCloud(laserCloudCornerFromLocal);
    ds_corner_.filter(*temp);
    laserCloudCornerFromLocal = temp;
    pcl::PointCloud<PointType>::Ptr temp2(new pcl::PointCloud<PointType>());
    ds_surf_.setInputCloud(laserCloudSurfFromLocal);
    ds_surf_.filter(*temp2);
    laserCloudSurfFromLocal = temp2;

    localMapID++;
  }

  bool ICPScanMatchGlobal(kylidar &kframe)
  {
    TicToc tc;
    tc.tic();
    CLOUD_PTR cloud_icp(new CLOUD());
    // *cloud_icp += *TransformPointCloud(kframe.corner, &initpose);
    *cloud_icp += *TransformPointCloud(kframe.surf, &initpose);

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(50);
    icp.setMaximumIterations(30);
    icp.setTransformationEpsilon(1e-4);
    icp.setEuclideanFitnessEpsilon(1e-4);
    icp.setRANSACIterations(0);

    icp.setInputSource(cloud_icp);
    icp.setInputTarget(surround_surf);
    CLOUD_PTR unused_result(new CLOUD());
    icp.align(*unused_result);

    if (icp.hasConverged() == false || icp.getFitnessScore() > 0.4)
    {
      std::cout << ANSI_COLOR_RED << "initial loc failed...,score: " << icp.getFitnessScore() << ANSI_COLOR_RESET << std::endl;
      return false;
    }
    Eigen::Affine3f correct_transform;
    correct_transform = icp.getFinalTransformation();
    Eigen::Matrix4d curr_pose = toMatrix(initpose);

    pose_in_map = correct_transform.matrix().cast<double>() * curr_pose;
    pose_in_map.block<3, 3>(0, 0) = Eigen::Quaterniond(pose_in_map.block<3, 3>(0, 0)).normalized().matrix();
    double tt = tc.toc();
    std::cout << "icp takes: " << tt << "ms" << std::endl;
    CLOUD_PTR output(new CLOUD);

    /*pcl::transformPointCloud(*kframe.cloud, *output, pose_in_map);
    sensor_msgs::PointCloud2 msg_target;
    pcl::toROSMsg(*output, msg_target);
    msg_target.header.stamp = ros::Time::now();
    msg_target.header.frame_id = "world";
    pub_surf_.publish(msg_target);*/

    return true;
  }

  Eigen::Matrix4d toMatrix(PointTypePose &p)
  {
    Eigen::Matrix4d odom = Eigen::Matrix4d::Identity();
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(p.roll, Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(p.pitch, Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(p.yaw, Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond rotation = yawAngle * pitchAngle * rollAngle;
    odom.block(0, 0, 3, 3) = rotation.toRotationMatrix();
    odom(0, 3) = p.x, odom(1, 3) = p.y, odom(2, 3) = p.z;
    return odom;
  }

  CLOUD_PTR TransformPointCloud(CLOUD_PTR cloudIn, PointTypePose *transformIn)
  {
    CLOUD_PTR cloudOut(new CLOUD());
    PointType *pointfrom;
    PointType pointTo;

    int cloudSize = cloudIn->points.size();
    cloudOut->resize(cloudSize);
    for (int i = 0; i < cloudSize; i++)
    {
      pointfrom = &cloudIn->points[i];
      float x1 = pointfrom->x;
      float y1 = cos(transformIn->roll) * pointfrom->y - sin(transformIn->roll) * pointfrom->z;
      float z1 = sin(transformIn->roll) * pointfrom->y + cos(transformIn->roll) * pointfrom->z;

      float x2 = cos(transformIn->pitch) * x1 + sin(transformIn->pitch) * z1;
      float y2 = y1;
      float z2 = -sin(transformIn->pitch) * x1 + cos(transformIn->pitch) * z1;

      pointTo.x = cos(transformIn->yaw) * x2 - sin(transformIn->yaw) * y2 + transformIn->x;
      pointTo.y = sin(transformIn->yaw) * x2 + cos(transformIn->yaw) * y2 + transformIn->y;
      pointTo.z = z2 + transformIn->z;
      pointTo.intensity = pointfrom->intensity;

      cloudOut->points[i] = pointTo;
    }
    return cloudOut;
  }

  bool extractSurroundKeyFrames(const PointType &p)
  {
    TicToc tc;
    tc.tic();
    std::cout << "-----extract surround keyframes ------ " << std::endl;
    std::vector<int> point_search_idx_;
    std::vector<float> point_search_dist_;
    double surround_search_radius_ = 5.0;
    kdtree_keyposes_3d_->radiusSearch(p, surround_search_radius_, point_search_idx_, point_search_dist_, 0);
    surround_surf->clear();
    surround_corner->clear();
    for (int i = 0; i < point_search_idx_.size(); ++i)
    {
      *surround_surf += *map.surf_keyframes_[i];
      *surround_corner += *map.corner_keyframes_[i];
    }
    ds_corner_.setInputCloud(surround_corner);
    ds_corner_.filter(*surround_corner);
    ds_surf_.setInputCloud(surround_surf);
    ds_surf_.filter(*surround_surf);
    double tt = tc.toc();
    std::cout << __FUNCTION__ << ",takes: " << tt << "ms" << std::endl;
  }

  bool loadmap()
  {
    std::cout << ANSI_COLOR_YELLOW << "file dir: " << filename << ANSI_COLOR_RESET << std::endl;
    CLOUD_PTR globalCornerCloud(new CLOUD);
    CLOUD_PTR globalSurfCloud(new CLOUD);

    std::string fn_poses_ = filename + "/trajectory.pcd";
    std::string fn_corner_ = filename + "/CornerMap.pcd";
    std::string fn_surf_ = filename + "/SurfMap.pcd";
    std::string fn_global_ = filename + "/GlobalMap.pcd";

    if (pcl::io::loadPCDFile(fn_poses_, *map.cloudKeyPoses3D_) == -1 ||
        pcl::io::loadPCDFile(fn_corner_, *globalCornerCloud) == -1 ||
        pcl::io::loadPCDFile(fn_surf_, *globalSurfCloud) == -1 ||
        pcl::io::loadPCDFile(fn_global_, *map.globalMapCloud_))
    {
      std::cout << ANSI_COLOR_RED << "couldn't load pcd file" << ANSI_COLOR_RESET << std::endl;
      return false;
    }

    map.corner_keyframes_.resize(map.cloudKeyPoses3D_->points.size());
    map.surf_keyframes_.resize(map.cloudKeyPoses3D_->points.size());
    for (int i = 0; i < map.cloudKeyPoses3D_->points.size(); ++i)
    {
      map.corner_keyframes_[i] = CLOUD_PTR(new CLOUD);
      map.surf_keyframes_[i] = CLOUD_PTR(new CLOUD);
    }

    for (int i = 0; i < globalCornerCloud->points.size(); ++i)
    {
      const auto &p = globalCornerCloud->points[i];
      map.corner_keyframes_[int(p.intensity)]->points.push_back(p);
    }
    for (int i = 0; i < globalSurfCloud->points.size(); ++i)
    {
      const auto &p = globalSurfCloud->points[i];
      map.surf_keyframes_[int(p.intensity)]->points.push_back(p);
    }
    ds_corner_.setInputCloud(globalCornerCloud);
    ds_corner_.filter(*map.globalCornerMapCloud_);
    std::cout << globalCornerCloud->size() << ", " << map.globalCornerMapCloud_->size() << std::endl;
    ds_surf_.setInputCloud(globalSurfCloud);
    ds_surf_.filter(*map.globalSurfMapCloud_);
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "LOC");
  ROS_INFO("\033[1;32m----> LOC Started.\033[0m");

  std::cout << "ROOT_DIR: " << root_dir << std::endl;
  // std::string command = "mkdir -p " + root_dir + "Log";
  // system(command.c_str());

  map_location *lol = new map_location();
  std::thread opt_thread(&map_location::run, lol);
  // lol->run();

  ros::spin();

  return 0;
}
