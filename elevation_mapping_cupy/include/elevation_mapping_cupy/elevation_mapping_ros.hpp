//
// Copyright (c) 2022, Takahiro Miki. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#pragma once

// STL
#include <iostream>
#include <mutex>

// Eigen
#include <Eigen/Dense>


// Pybind
#include <pybind11/embed.h>  // everything needed for embedding
#include <pybind11/eigen.h>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
// #include <tf2/utils.h>
// #include <tf2/tf2/convert.h>
// #include <tf2/convert.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2/convert.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2_eigen/tf2_eigen.h>


// Grid Map
#include <grid_map_msgs/srv/get_grid_map.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

// PCL
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/projection_matrix.h>

#include <elevation_map_msgs/srv/check_safety.hpp>
#include <elevation_map_msgs/srv/initialize.hpp>
#include <elevation_map_msgs/msg/statistics.hpp>

#include "elevation_mapping_cupy/elevation_mapping_wrapper.hpp"

namespace py = pybind11;

namespace elevation_mapping_cupy {

class ElevationMappingNode : public rclcpp::Node {
 public:
  ElevationMappingNode();

 private:
  void readParameters();
  void setupMapPublishers();
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);
  void publishAsPointCloud(const grid_map::GridMap& map) const;
  bool getSubmap(grid_map_msgs::srv::GetGridMap::Request::SharedPtr request, grid_map_msgs::srv::GetGridMap::Response::SharedPtr response);
  bool checkSafety(elevation_map_msgs::srv::CheckSafety::Request::SharedPtr request, elevation_map_msgs::srv::CheckSafety::Response::SharedPtr response);
  bool initializeMap(elevation_map_msgs::srv::Initialize::Request::SharedPtr request, elevation_map_msgs::srv::Initialize::Response::SharedPtr response);
  bool clearMap(std_srvs::srv::Empty::Request::SharedPtr request, std_srvs::srv::Empty::Response::SharedPtr response);
  bool clearMapWithInitializer(std_srvs::srv::Empty::Request::SharedPtr request, std_srvs::srv::Empty::Response::SharedPtr response);
  bool setPublishPoint(std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response);
  void updatePose();
  void updateVariance();
  void updateTime();
  void updateGridMap();
  void publishNormalAsArrow(const grid_map::GridMap& map) const;
  void initializeWithTF();
  void publishMapToOdom(double error);
  void publishStatistics();
  void publishMapOfIndex(int index);
  visualization_msgs::msg::Marker vectorToArrowMarker(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    const int id) const;


  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> pointcloudSubs_;
  std::vector<rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr> mapPubs_;
  
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr alivePub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointPub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr normalPub_;
  rclcpp::Publisher<elevation_map_msgs::msg::Statistics>::SharedPtr statisticsPub_;
  rclcpp::Service<grid_map_msgs::srv::GetGridMap>::SharedPtr rawSubmapService_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clearMapService_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clearMapWithInitializerService_;
  rclcpp::Service<elevation_map_msgs::srv::Initialize>::SharedPtr initializeMapService_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr setPublishPointService_;
  rclcpp::Service<elevation_map_msgs::srv::CheckSafety>::SharedPtr checkSafetyService_;
  rclcpp::TimerBase::SharedPtr updateVarianceTimer_;
  rclcpp::TimerBase::SharedPtr updateTimeTimer_;
  rclcpp::TimerBase::SharedPtr updatePoseTimer_;
  rclcpp::TimerBase::SharedPtr updateGridMapTimer_;
  rclcpp::TimerBase::SharedPtr publishStatisticsTimer_;
  rclcpp::Time lastStatisticsPublishedTime_;
  // tf2_ros::Buffer transformListener_;
  // tf2_ros::TransformBroadcaster tfBroadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  ElevationMappingWrapper map_;
  std::string mapFrameId_;
  std::string correctedMapFrameId_;
  std::string baseFrameId_;

  // map topics info
  std::vector<std::vector<std::string>> map_topics_;
  std::vector<std::vector<std::string>> map_layers_;
  std::vector<std::vector<std::string>> map_basic_layers_;
  std::set<std::string> map_layers_all_;
  std::set<std::string> map_layers_sync_;
  std::vector<double> map_fps_;
  std::set<double> map_fps_unique_;
  std::vector<rclcpp::TimerBase::SharedPtr> mapTimers_;

  std::vector<std::string> initialize_frame_id_;
  std::vector<double> initialize_tf_offset_;
  std::string initializeMethod_;

  Eigen::Vector3d lowpassPosition_;
  Eigen::Vector4d lowpassOrientation_;

  std::mutex mapMutex_;  // protects gridMap_
  grid_map::GridMap gridMap_;
  std::atomic_bool isGridmapUpdated_;  // needs to be atomic (read is not protected by mapMutex_)

  std::mutex errorMutex_; // protects positionError_, and orientationError_
  double positionError_;
  double orientationError_;

  double positionAlpha_;
  double orientationAlpha_;

  double recordableFps_;
  std::atomic_bool enablePointCloudPublishing_;
  bool enableNormalArrowPublishing_;
  bool enableDriftCorrectedTFPublishing_;
  bool useInitializerAtStart_;
  double initializeTfGridSize_;
  std::atomic_int pointCloudProcessCounter_;
};

}  // namespace elevation_mapping_cupy
