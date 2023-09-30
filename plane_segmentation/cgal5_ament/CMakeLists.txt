cmake_minimum_required(VERSION 3.8)
project(cgal5_ament)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

include(ExternalProject)

set(CGAL_VERSION 5.3)

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/install/include)

ExternalProject_Add(cgal
  URL https://github.com/CGAL/cgal/archive/refs/tags/v${CGAL_VERSION}.tar.gz
  PREFIX ${CMAKE_BINARY_DIR}/external
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/install
    -DCMAKE_BUILD_TYPE:STRING=Release
  BUILD_COMMAND $(MAKE)
  INSTALL_COMMAND $(MAKE) install
)

ament_export_include_directories(${CMAKE_BINARY_DIR}/install/include)
ament_export_libraries(cgal)

ament_package()
