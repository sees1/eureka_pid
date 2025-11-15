cmake_minimum_required(VERSION 3.11)
project(eureka_pid)

find_package(ament_cmake REQUIRED)
# add_definitions(-DDEBUG)
include_directories(
  include
)

set(dependencies_pkgs
  rclcpp
  nav2_common
  pluginlib
  tf2
  geometry_msgs
  visualization_msgs
  nav_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav2_mppi_controller
  tf2_geometry_msgs
  tf2_eigen
  tf2_ros
)

foreach(pkg IN LISTS dependencies_pkgs)
  find_package(${pkg} REQUIRED)
endforeach()

nav2_package()

include(CheckCXXCompilerFlag)

# If building one the same hardware to be deployed on, try `-march=native`!

add_library(eureka_controller SHARED
  src/controller.cpp
  src/path_handler.cpp
  src/pid.cpp
)

add_executable(testm
  src/main.cpp)

set(libraries eureka_controller)

foreach(lib IN LISTS libraries)
  ament_target_dependencies(${lib} ${dependencies_pkgs})
endforeach()

ament_target_dependencies(testm ${dependencies_pkgs})

install(TARGETS eureka_controller testm
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/eureka_jerk
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_libraries(${libraries})
ament_export_dependencies(${dependencies_pkgs})
ament_export_include_directories(include)
pluginlib_export_plugin_description_file(nav2_core controller.xml)

ament_package()
