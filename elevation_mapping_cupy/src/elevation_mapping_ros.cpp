//
// Copyright (c) 2022, Takahiro Miki. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "elevation_mapping_cupy/elevation_mapping_ros.hpp"

// Pybind
#include <pybind11/eigen.h>

// ROS
#include <geometry_msgs/msg/point32.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2_eigen/tf2_eigen.h>

// PCL
#include <pcl/common/projection_matrix.h>

#include <elevation_map_msgs/msg/statistics.hpp>

#include <chrono>

namespace elevation_mapping_cupy {

ElevationMappingNode::ElevationMappingNode()
  : rclcpp::Node("elevation_mapping_wrapper"),
    lowpassPosition_(0, 0, 0),
    lowpassOrientation_(0, 0, 0, 1),
    positionError_(0),
    orientationError_(0),
    positionAlpha_(0.1),
    orientationAlpha_(0.1),
    enablePointCloudPublishing_(false),
    isGridmapUpdated_(false) {
    
  tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);// ROS2构造TransformBroadcaster
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(this);
  tfBuffer_ = std::make_shared<tf2_ros::Buffer>(tfListener_);

  map_.initialize();
  std::string pose_topic, map_frame;
  std::vector<std::string> pointcloud_topics;
  std::vector<std::string> map_topics;
  double recordableFps, updateVarianceFps, timeInterval, updatePoseFps, updateGridMapFps, publishStatisticsFps;
  bool enablePointCloudPublishing(false);

  // 
  rclcpp::Parameter param;
  if (this->get_parameter("publishers", param)) {
    // auto publishers = param.as<YourPublisherType>(); // Replace YourPublisherType with the actual data type you expect.
    // Now you have the 'publishers' variable with the parameter value.
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to get 'publishers' parameter");
  }

  pointcloud_topics = this->declare_parameter<std::vector<std::string>>("pointcloud_topics", {"points"});
  initialize_frame_id_ = this->declare_parameter<std::vector<std::string>>("initialize_frame_id", {"base"});
  initialize_tf_offset_ = this->declare_parameter<std::vector<double>>("initialize_tf_offset", {0.0});
  pose_topic = this->declare_parameter<std::string>("pose_topic", "pose");
  mapFrameId_ = this->declare_parameter<std::string>("map_frame", "map");
  baseFrameId_ = this->declare_parameter<std::string>("base_frame", "base");
  correctedMapFrameId_ = this->declare_parameter<std::string>("corrected_map_frame", "corrected_map");
  initializeMethod_ = this->declare_parameter<std::string>("initialize_method", "cubic");
  positionAlpha_ = this->declare_parameter<double>("position_lowpass_alpha", 0.2);
  orientationAlpha_ = this->declare_parameter<double>("orientation_lowpass_alpha", 0.2);
  recordableFps = this->declare_parameter<double>("recordable_fps", 3.0);
  updateVarianceFps = this->declare_parameter<double>("update_variance_fps", 1.0);
  timeInterval = this->declare_parameter<double>("time_interval", 0.1);
  updatePoseFps = this->declare_parameter<double>("update_pose_fps", 10.0);
  initializeTfGridSize_ = this->declare_parameter<double>("initialize_tf_grid_size", 0.5);
  updateGridMapFps = this->declare_parameter<double>("map_acquire_fps", 5.0);
  publishStatisticsFps = this->declare_parameter<double>("publish_statistics_fps", 1.0);
  enablePointCloudPublishing = this->declare_parameter<bool>("enable_pointcloud_publishing", false);
  enableNormalArrowPublishing_ = this->declare_parameter<bool>("enable_normal_arrow_publishing", false);
  enableDriftCorrectedTFPublishing_ = this->declare_parameter<bool>("enable_drift_corrected_TF_publishing", false);
  useInitializerAtStart_ = this->declare_parameter<bool>("use_initializer_at_start", false);

  enablePointCloudPublishing_ = enablePointCloudPublishing;

  for (const auto& pointcloud_topic : pointcloud_topics) {
    auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, 1, &ElevationMappingNode::pointcloudCallback);
    pointcloudSubs_.push_back(sub);
  }

  // register map publishers
  for (auto itr = publishers.begin(); itr != publishers.end(); ++itr) {
    // parse params
    std::string topic_name = itr->first;
    std::vector<std::string> layers_list;
    std::vector<std::string> basic_layers_list;
    auto layers = itr->second["layers"];
    auto basic_layers = itr->second["basic_layers"];
    double fps = itr->second["fps"];

    if (fps > updateGridMapFps) {
      RCLCPP_WARN(
          this->get_logger(),
          "[ElevationMappingCupy] fps for topic %s is larger than map_acquire_fps (%f > %f). The topic data will be only updated at %f "
          "fps.",
          topic_name.c_str(), fps, updateGridMapFps, updateGridMapFps);
    }

    for (int32_t i = 0; i < layers.size(); ++i) {
      layers_list.push_back(static_cast<std::string>(layers[i]));
    }

    for (int32_t i = 0; i < basic_layers.size(); ++i) {
      basic_layers_list.push_back(static_cast<std::string>(basic_layers[i]));
    }

    // make publishers
    auto pub = this->create_publisher<grid_map_msgs::msg::GridMap>(
        topic_name, 1);
    mapPubs_.push_back(pub);

    // register map layers
    map_layers_.push_back(layers_list);
    map_basic_layers_.push_back(basic_layers_list);

    // register map fps
    map_fps_.push_back(fps);
    map_fps_unique_.insert(fps);
  }
  setupMapPublishers();

  pointPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("elevation_map_points", 1);
  alivePub_ = this->create_publisher<std_msgs::msg::Empty>("alive", 1);
  normalPub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("normal", 1);
  statisticsPub_ = this->create_publisher<elevation_map_msgs::msg::Statistics>("statistics", 1);

  gridMap_.setFrameId(mapFrameId_);
  rawSubmapService_ = this->create_service<grid_map_msgs::srv::GetGridMap>("get_raw_submap", std::bind(&ElevationMappingNode::getSubmap, this, std::placeholders::_1, std::placeholders::_2));
  clearMapService_ = this->create_service<std_srvs::srv::Empty>("clear_map", std::bind(&ElevationMappingNode::clearMap, this, std::placeholders::_1, std::placeholders::_2));
  initializeMapService_ = this->create_service<elevation_map_msgs::srv::Initialize>("initialize", std::bind(&ElevationMappingNode::initializeMap, this, std::placeholders::_1, std::placeholders::_2));
  clearMapWithInitializerService_ = this->create_service<std_srvs::srv::Empty>("clear_map_with_initializer", std::bind(&ElevationMappingNode::clearMapWithInitializer, this, std::placeholders::_1, std::placeholders::_2));
  setPublishPointService_ = this->create_service<std_srvs::srv::SetBool>("set_publish_points", std::bind(&ElevationMappingNode::setPublishPoint, this, std::placeholders::_1, std::placeholders::_2));
  checkSafetyService_ = this->create_service<elevation_map_msgs::srv::CheckSafety>("check_safety", std::bind(&ElevationMappingNode::checkSafety, this, std::placeholders::_1, std::placeholders::_2));

  if (updateVarianceFps > 0) {
    double duration = 1.0 / (updateVarianceFps + 0.00001);
    updateVarianceTimer_ = this->create_timer(std::chrono::duration<double>(duration), std::bind(&ElevationMappingNode::updateVariance, this));
  }
  if (timeInterval > 0) {
    double duration = timeInterval;
    updateTimeTimer_ = this->create_timer(std::chrono::duration<double>(duration), std::bind(&ElevationMappingNode::updateTime, this));
  }
  if (updatePoseFps > 0) {
    double duration = 1.0 / (updatePoseFps + 0.00001);
    updatePoseTimer_ = this->create_timer(std::chrono::duration<double>(duration), std::bind(&ElevationMappingNode::updatePose, this));
  }
  if (updateGridMapFps > 0) {
    double duration = 1.0 / (updateGridMapFps + 0.00001);
    updateGridMapTimer_ = this->create_timer(std::chrono::duration<double>(duration), std::bind(&ElevationMappingNode::updateGridMap, this));
  }
  if (publishStatisticsFps > 0) {
    double duration = 1.0 / (publishStatisticsFps + 0.00001);
    publishStatisticsTimer_ = this->create_timer(std::chrono::duration<double>(duration), std::bind(&ElevationMappingNode::publishStatistics, this));
  }
  lastStatisticsPublishedTime_ = this->now();
  RCLCPP_INFO(this->get_logger(), "[ElevationMappingCupy] finish initialization");
}

// setup map publishers
void ElevationMappingNode::setupMapPublishers() {
  // Find the layers with highest fps.
  float max_fps = -1;
  // create timers for each unique map frequencies
  for (auto fps : map_fps_unique_) {
    // which publisher to call in the timer callback
    std::vector<int> indices;
    // if this fps is max, update the map layers.
    if (fps >= max_fps) {
      max_fps = fps;
      map_layers_all_.clear();
    }
    for (int i = 0; i < map_fps_.size(); i++) {
      if (map_fps_[i] == fps) {
        indices.push_back(i);
        // if this fps is max, add layers
        if (fps >= max_fps) {
          for (const auto layer : map_layers_[i]) {
            map_layers_all_.insert(layer);
          }
        }
      }
    }
    // callback funtion.
    // It publishes to specific topics.
    auto cb = [this, indices]() {
      for (int i : indices) {
        publishMapOfIndex(i);
      }
    };
    double duration = 1.0 / (fps + 0.00001);
    mapTimers_.push_back(this->create_timer(std::chrono::duration<double>(duration), cb));
  
  }
}

void ElevationMappingNode::publishMapOfIndex(int index) {
  // publish the map layers of index
  if (!isGridmapUpdated_) {
    return;
  }
  grid_map_msgs::msg::GridMap msg;
  std::vector<std::string> layers;

  {  // need continuous lock between adding layers and converting to message. Otherwise updateGridmap can reset the data not in
     // map_layers_all_
    std::lock_guard<std::mutex> lock(mapMutex_);
    for (const auto& layer : map_layers_[index]) {
      const bool is_layer_in_all = map_layers_all_.find(layer) != map_layers_all_.end();
      if (is_layer_in_all && gridMap_.exists(layer)) {
        layers.push_back(layer);
      } else if (map_.exists_layer(layer)) {
        // if there are layers which is not in the syncing layer.
        ElevationMappingWrapper::RowMatrixXf map_data;
        map_.get_layer_data(layer, map_data);
        gridMap_.add(layer, map_data);
        layers.push_back(layer);
      }
    }
    if (layers.empty()) {
      return;
    }

    grid_map::GridMapRosConverter::toMessage(gridMap_, layers, msg);
    //  = grid_map::GridMapRosConverter::toMessage(gridMap_, layers);
  }

  msg.basic_layers = map_basic_layers_[index];
  mapPubs_[index]->publish(msg);
}

void ElevationMappingNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
  auto start = rclcpp::Clock().now(); // 使用ROS 2的时钟获取当前时间

  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*cloud, pcl_pc);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc, *pointCloud);

  // 使用ROS 2的TransformListener
  geometry_msgs::msg::TransformStamped transformTf;
  std::string sensorFrameId = cloud->header.frame_id;
  auto timeStamp = cloud->header.stamp;

  try {
    transformTf = tfBuffer_->lookupTransform(mapFrameId_, sensorFrameId, timeStamp);
    Eigen::Affine3d transformationSensorToMap;
    tf2::fromMsg(transformTf.transform, transformationSensorToMap);
    
    double positionError{0.0};
    double orientationError{0.0};
    
    {
      std::lock_guard<std::mutex> lock(errorMutex_);
      positionError = positionError_;
      orientationError = orientationError_;
    }

    map_.input(pointCloud, transformationSensorToMap.rotation(), transformationSensorToMap.translation(), positionError, orientationError);

    if (enableDriftCorrectedTFPublishing_) {
      publishMapToOdom(map_.get_additive_mean_error());
    }

    RCLCPP_DEBUG(this->get_logger(), "ElevationMap processed a point cloud (%i points) in %f sec.", static_cast<int>(pointCloud->size()),
                 (rclcpp::Clock().now() - start).seconds());
    RCLCPP_DEBUG(this->get_logger(), "positionError: %f ", positionError);
    RCLCPP_DEBUG(this->get_logger(), "orientationError: %f ", orientationError);
    // This is used for publishing as statistics.
    pointCloudProcessCounter_++;
  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
  }
}

void ElevationMappingNode::updatePose() {
  tf2::Transform transform_tf;
  const auto timeStamp = this->now();
  try {
    // 使用tf2_ros::Buffer查询变换
    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg = tfBuffer_->lookupTransform(mapFrameId_, baseFrameId_, timeStamp, tf2::durationFromSec(1.0));

    // 获取位姿信息
    tf2::fromMsg(transform_msg.transform, transform_tf);

  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    return;
  }

  // 获取位姿中的位置和旋转信息
  tf2::Vector3 position = transform_tf.getOrigin();
  tf2::Quaternion orientation = transform_tf.getRotation();

  // This is to check if the robot is moving. If the robot is not moving, drift compensation is disabled to avoid creating artifacts.
  Eigen::Vector3d position3(position.getX(), position.getY(), position.getZ());
  map_.move_to(position3);
  Eigen::Vector4d orientation4(orientation.getX(), orientation.getY(), orientation.getZ(), orientation.getW());
  lowpassPosition_ = positionAlpha_ * position3 + (1 - positionAlpha_) * lowpassPosition_;
  lowpassOrientation_ = orientationAlpha_ * orientation4 + (1 - orientationAlpha_) * lowpassOrientation_;
  {
    std::lock_guard<std::mutex> lock(errorMutex_);
    positionError_ = (position3 - lowpassPosition_).norm();
    orientationError_ = (orientation4 - lowpassOrientation_).norm();
  }

  if (useInitializerAtStart_) {
    RCLCPP_INFO(this->get_logger(),"Clearing map with initializer.");
    initializeWithTF();
    useInitializerAtStart_ = false;
  }
}

void ElevationMappingNode::publishAsPointCloud(const grid_map::GridMap& map) const {
  sensor_msgs::msg::PointCloud2 msg;
  grid_map::GridMapRosConverter::toPointCloud(map, "elevation", msg);
  pointPub_->publish(msg);
}

bool ElevationMappingNode::getSubmap(
  const std::shared_ptr<grid_map_msgs::srv::GetGridMap::Request> request,
  std::shared_ptr<grid_map_msgs::srv::GetGridMap::Response> response)
{
  std::string requestedFrameId = request->frame_id;
  Eigen::Isometry3d transformationOdomToMap;
  grid_map::Position requestedSubmapPosition(request->position_x, request->position_y);
  if (requestedFrameId != mapFrameId_) {
    geometry_msgs::msg::TransformStamped transformTf;
    const auto& timeStamp = now();
    try {
      transformTf = tfBuffer_->lookupTransform(mapFrameId_, requestedFrameId, timeStamp);
      tf2::fromMsg(transformTf.transform, transformationOdomToMap);
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(this->get_logger(),"%s", ex.what());
      return false;
    }
    Eigen::Vector3d p(request->position_x, request->position_y, 0);
    Eigen::Vector3d mapP = transformationOdomToMap.inverse() * p;
    requestedSubmapPosition.x() = mapP.x();
    requestedSubmapPosition.y() = mapP.y();
  }
  grid_map::Length requestedSubmapLength(request->length_x, request->length_y);
  RCLCPP_DEBUG(this->get_logger(),"Elevation submap request: Position x=%f, y=%f, Length x=%f, y=%f.",
    requestedSubmapPosition.x(), requestedSubmapPosition.y(),
    requestedSubmapLength(0), requestedSubmapLength(1));

  bool isSuccess;
  grid_map::Index index; // ?
  grid_map::GridMap subMap;
  {
    std::lock_guard<std::mutex> lock(mapMutex_);
    subMap = gridMap_.getSubmap(requestedSubmapPosition, requestedSubmapLength, isSuccess);
    // subMap = gridMap_.getSubmap(requestedSubmapPosition, requestedSubmapLength, index, isSuccess);
  }
  const auto& length = subMap.getLength();
  if (requestedFrameId != mapFrameId_) {
    subMap = subMap.getTransformedMap(transformationOdomToMap, "elevation", requestedFrameId);
  }

  if (request->layers.empty()) {
    grid_map::GridMapRosConverter::toMessage(subMap, response->map);
  } else {
    std::vector<std::string> layers;
    for (const auto& layer : request->layers) {
      layers.push_back(layer);
    }
    grid_map::GridMapRosConverter::toMessage(subMap, layers, response->map);
  }
  return isSuccess;
}

bool ElevationMappingNode::clearMap(
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*response*/)
{
  RCLCPP_INFO(this->get_logger(),"Clearing map.");
  map_.clear();
  return true;
}

bool ElevationMappingNode::clearMapWithInitializer(
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*response*/)
{
  RCLCPP_INFO(this->get_logger(),"Clearing map with initializer.");
  map_.clear();
  initializeWithTF();
  return true;
}

void ElevationMappingNode::initializeWithTF() {
  std::vector<Eigen::Vector3d> points;
  const auto& timeStamp = this->now();
  int i = 0;
  Eigen::Vector3d p;
  for (const auto& frame_id : initialize_frame_id_) {
    // Get tf from map frame to tf frame
    Eigen::Affine3d transformationBaseToMap;
    geometry_msgs::msg::TransformStamped transformTf;
    try {
      transformTf = tfBuffer_->lookupTransform(mapFrameId_, frame_id, timeStamp);
      tf2::fromMsg(transformTf.transform, transformationBaseToMap);
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(this->get_logger(),"%s", ex.what());
      return;
    }
    p = transformationBaseToMap.translation();
    p.z() += initialize_tf_offset_[i];
    points.push_back(p);
    i++;
  }
  if (!points.empty() && points.size() < 3) {
    points.emplace_back(p + Eigen::Vector3d(initializeTfGridSize_, initializeTfGridSize_, 0));
    points.emplace_back(p + Eigen::Vector3d(-initializeTfGridSize_, initializeTfGridSize_, 0));
    points.emplace_back(p + Eigen::Vector3d(initializeTfGridSize_, -initializeTfGridSize_, 0));
    points.emplace_back(p + Eigen::Vector3d(-initializeTfGridSize_, -initializeTfGridSize_, 0));
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Initializing map with points using " << initializeMethod_);
  map_.initializeWithPoints(points, initializeMethod_);
}
bool ElevationMappingNode::checkSafety(const elevation_map_msgs::srv::CheckSafety::Request::SharedPtr request,
                                      elevation_map_msgs::srv::CheckSafety::Response::SharedPtr response){
  for (const auto& polygonstamped : request->polygons) {
    if (polygonstamped.polygon.points.empty()) {
      continue;
    }
    std::vector<Eigen::Vector2d> polygon;
    std::vector<Eigen::Vector2d> untraversable_polygon;
    Eigen::Vector3d result;
    result.setZero();
    const auto& polygonFrameId = polygonstamped.header.frame_id;
    const auto& timeStamp = polygonstamped.header.stamp;
    double polygon_z = polygonstamped.polygon.points[0].z;

    // Get tf from map frame to polygon frame
    if (mapFrameId_ != polygonFrameId) {
      Eigen::Affine3d transformationBaseToMap;
      geometry_msgs::msg::TransformStamped transformTf;
      try {
        transformTf = tfBuffer_->lookupTransform(mapFrameId_, polygonFrameId, timeStamp);
        tf2::fromMsg(transformTf.transform, transformationBaseToMap);
      } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(),"%s", ex.what());
        return false;
      }
      for (const auto& p : polygonstamped.polygon.points) {
        const auto& pvector = Eigen::Vector3d(p.x, p.y, p.z);
        const auto transformed_p = transformationBaseToMap * pvector;
        polygon.emplace_back(Eigen::Vector2d(transformed_p.x(), transformed_p.y()));
      }
    } else {
      for (const auto& p : polygonstamped.polygon.points) {
        polygon.emplace_back(Eigen::Vector2d(p.x, p.y));
      }
    }

    map_.get_polygon_traversability(polygon, result, untraversable_polygon);

    geometry_msgs::msg::PolygonStamped untraversable_polygonstamped;
    untraversable_polygonstamped.header.stamp = this->now();
    untraversable_polygonstamped.header.frame_id = mapFrameId_;
    for (const auto& p : untraversable_polygon) {
      geometry_msgs::msg::Point32 point;
      point.x = static_cast<float>(p.x());
      point.y = static_cast<float>(p.y());
      point.z = static_cast<float>(polygon_z);
      untraversable_polygonstamped.polygon.points.push_back(point);
    }
    // traversability_result;
    response->is_safe.push_back(bool(result[0] > 0.5));
    response->traversability.push_back(result[1]);
    response->untraversable_polygons.push_back(untraversable_polygonstamped);
  }
  return true;
}

bool ElevationMappingNode::setPublishPoint(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                           std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    enablePointCloudPublishing_ = request->data;
    response->success = true;
    return true;
}

void ElevationMappingNode::updateVariance() {
  map_.update_variance();
}

void ElevationMappingNode::updateTime() {
  map_.update_time();
}

void ElevationMappingNode::publishStatistics()
{
    rclcpp::Time now = this->now();
    double dt = (now - lastStatisticsPublishedTime_).seconds();
    lastStatisticsPublishedTime_ = now;
    elevation_map_msgs::msg::Statistics msg;
    msg.header.stamp = now;
    if (dt > 0.0) {
        msg.pointcloud_process_fps = pointCloudProcessCounter_ / dt;
    }
    pointCloudProcessCounter_ = 0;
    statisticsPub_->publish(msg);
}

void ElevationMappingNode::updateGridMap()
{
    std::vector<std::string> layers(map_layers_all_.begin(), map_layers_all_.end());
    std::lock_guard<std::mutex> lock(mapMutex_);
    map_.get_grid_map(gridMap_, layers);
    gridMap_.setTimestamp(this->now().nanoseconds());
    alivePub_->publish(std_msgs::msg::Empty());

    // Mostly debug purpose
    if (enablePointCloudPublishing_) {
        publishAsPointCloud(gridMap_);
    }
    if (enableNormalArrowPublishing_) {
        publishNormalAsArrow(gridMap_);
    }
    isGridmapUpdated_ = true;
}

bool ElevationMappingNode::initializeMap(
  const elevation_map_msgs::srv::Initialize::Request::SharedPtr request,
  elevation_map_msgs::srv::Initialize::Response::SharedPtr response) {
  // If initialize method is points
  if (request->type == request->POINTS) {
    std::vector<Eigen::Vector3d> points;
    for (const auto& point : request->points) {
      const auto& pointFrameId = point.header.frame_id;
      const auto& timeStamp = point.header.stamp;
      const auto& pvector = Eigen::Vector3d(point.point.x, point.point.y, point.point.z);

      // Get tf from map frame to points' frame
      if (mapFrameId_ != pointFrameId) {
        Eigen::Affine3d transformationBaseToMap;
        geometry_msgs::msg::TransformStamped transformTf;
        try {
          transformTf = tfBuffer_->lookupTransform(mapFrameId_, pointFrameId, timeStamp);
          tf2::fromMsg(transformTf.transform, transformationBaseToMap);
        } catch (tf2::TransformException& ex) {
          RCLCPP_ERROR(this->get_logger(),"%s", ex.what());
          return false;
        }
        const auto transformed_p = transformationBaseToMap * pvector;
        points.push_back(transformed_p);
      } else {
        points.push_back(pvector);
      }
    }
    std::string method;
    switch (request->method) {
      case elevation_map_msgs::srv::Initialize::Request::NEAREST:  //哪种？request->NEAREST
        method = "nearest";
        break;
      case elevation_map_msgs::srv::Initialize::Request::LINEAR:
        method = "linear";
        break;
      case elevation_map_msgs::srv::Initialize::Request::CUBIC:
        method = "cubic";
        break;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Initializing map with points using " << method);
    map_.initializeWithPoints(points, method);
  }
  response->success = true;
  return true;
}

void ElevationMappingNode::publishNormalAsArrow(const grid_map::GridMap& map) const {
  auto startTime = this->now();

  const auto& normalX = map["normal_x"];
  const auto& normalY = map["normal_y"];
  const auto& normalZ = map["normal_z"];
  double scale = 0.1;

  visualization_msgs::msg::MarkerArray markerArray;
  // For each cell in map.
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    if (!map.isValid(*iterator, "elevation")) {
      continue;
    }
    grid_map::Position3 p;
    map.getPosition3("elevation", *iterator, p);
    Eigen::Vector3d start = p;
    const auto i = iterator.getLinearIndex();
    Eigen::Vector3d normal(normalX(i), normalY(i), normalZ(i));
    Eigen::Vector3d end = start + normal * scale;
    if (normal.norm() < 0.1) {
      continue;
    }
    markerArray.markers.push_back(vectorToArrowMarker(start, end, i));
  }
  normalPub_->publish(markerArray);
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1.0, "publish as normal in %f sec.", (this->now() - startTime).seconds());

}

visualization_msgs::msg::Marker ElevationMappingNode::vectorToArrowMarker(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
                                                                     const int id) const {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = mapFrameId_;
  marker.header.stamp = this->now();
  marker.ns = "normal";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.points.resize(2);
  marker.points[0].x = start.x();
  marker.points[0].y = start.y();
  marker.points[0].z = start.z();
  marker.points[1].x = end.x();
  marker.points[1].y = end.y();
  marker.points[1].z = end.z();
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.scale.z = 0.01;
  marker.color.a = 1.0;  // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  return marker;
}

void ElevationMappingNode::publishMapToOdom(double error) {
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = this->now();
  transformStamped.header.frame_id = mapFrameId_;
  transformStamped.child_frame_id = correctedMapFrameId_;
  transformStamped.transform.translation.x = 0.0;
  transformStamped.transform.translation.y = 0.0;
  transformStamped.transform.translation.z = error;
  transformStamped.transform.rotation.x = 0.0;
  transformStamped.transform.rotation.y = 0.0;
  transformStamped.transform.rotation.z = 0.0;
  transformStamped.transform.rotation.w = 1.0;
  tfBroadcaster_->sendTransform(transformStamped);
}

}  // namespace elevation_mapping_cupy
