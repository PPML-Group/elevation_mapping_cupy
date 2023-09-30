//
// Copyright (c) 2022, Takahiro Miki. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

// Pybind
#include <pybind11/embed.h>  // everything needed for embedding

// ROS 2 C++ Includes
#include <rclcpp/rclcpp.hpp>

#include "elevation_mapping_cupy/elevation_mapping_ros.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<elevation_mapping_cupy::ElevationMappingNode>();

  // Create a separate thread for the ROS 2 node
  // std::thread([&node]() {
  //   rclcpp::spin(node);
  // }).detach();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}