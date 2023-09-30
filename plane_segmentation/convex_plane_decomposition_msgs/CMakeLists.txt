cmake_minimum_required(VERSION 3.8)
project(convex_plane_decomposition_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoundingBox2d.msg"
  "msg/Point2d.msg"
  "msg/Polygon2d.msg"
  "msg/PolygonWithHoles2d.msg"
  "msg/PlanarRegion.msg"
  "msg/PlanarTerrain.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs grid_map_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
