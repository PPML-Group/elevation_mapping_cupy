cmake_minimum_required(VERSION 3.8)
project(elevation_mapping_cupy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(pybind11 CONFIG REQUIRED)
MESSAGE( [MAIN] "Found pybind11 v${pybind11_VERSION}: ${pybind11_INCLUDE_DIRS}")
MESSAGE( [Main] " pybind11_INCLUDE_DIRS = ${pybind11_INCLUDE_DIRS}")
MESSAGE( [Main] " pybind11_LIBRARIES = ${pybind11_LIBRARIES}")

find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(elevation_map_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(ament_cmake_python REQUIRED)

# To solve: Unknown CMake command “python3_add_library”.
find_package(Python3 REQUIRED Development)
find_package(Python3 REQUIRED Interpreter)
find_package(PythonInterp 3 REQUIRED)
# find_package(PythonLibs 3 REQUIRED)
if(PYTHONLIBS_FOUND)
  message(STATUS "Using Python Libraries at: " ${PYTHON_LIBRARIES})
  message(STATUS "Using Python include directories at: " ${PYTHON_INCLUDE_DIRS})
else()
  message(WARNING "Could not find Python Libraries")
endif()

set(dependencies
  rclcpp
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  std_msgs
  std_srvs
  builtin_interfaces
  geometry_msgs
  sensor_msgs
  elevation_map_msgs
  grid_map_msgs
  grid_map_ros
  pcl_ros
)

include_directories(
  include
  ${PYTHON_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  # ${pybind11_INCLUDE_DIRS} # 
)

add_library(elevation_mapping_ros
    src/elevation_mapping_wrapper.cpp
    src/elevation_mapping_ros.cpp)

ament_target_dependencies(elevation_mapping_ros ${dependencies})
# target_link_libraries(elevation_mapping_ros ${PYTHON_LIBRARIES} pybind11::module pybind11::embed)
target_link_libraries(elevation_mapping_ros ${PYTHON_LIBRARIES})
add_executable(elevation_mapping_node src/elevation_mapping_node.cpp)
target_link_libraries(elevation_mapping_node elevation_mapping_ros)

# ament_python_install_package(${PROJECT_NAME})
# install(PROGRAMS
#   scripts/custom_kernels.py
#   scripts/elevation_mapping.py
#   scripts/map_initializer.py
#   scripts/parameter.py
#   scripts/traversability_filter.py
#   scripts/traversability_polygon.py
#   scripts/plugins/inpainting.py
#   scripts/plugins/min_filter.py
#   scripts/plugins/plugin_manager.py
#   scripts/plugins/smooth_filter.py
#   DESTINATION lib/${PROJECT_NAME}
# )

install(TARGETS 
  elevation_mapping_node 
  DESTINATION lib/${PROJECT_NAME}
  # ARCHIVE DESTINATION lib/${PROJECT_NAME}
  # LIBRARY DESTINATION lib/${PROJECT_NAME}
  # RUNTIME DESTINATION bin/${PROJECT_NAME}
)

install(DIRECTORY 
  scripts
  launch 
  config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
