cmake_minimum_required(VERSION 3.5)
project(rrt_plugin)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_core REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(PACKAGE_INCLUDE_DEPENDENCIES
  ament_cmake
  ament_cmake_ros
  rclcpp
  rclcpp_action
  tf2_geometry_msgs
  tf2_ros
  moveit_core
  rviz_visual_tools
  moveit_visual_tools
  moveit_ros_planning_interface
  interactive_markers
  tf2_geometry_msgs
  moveit_ros_planning
  pluginlib
  Eigen3
  Boost
  control_msgs
  moveit_servo
)
file(GLOB_RECURSE native_srcs src/*.cpp)
add_library(rrt_plugin ${native_srcs})

target_include_directories(rrt_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(rrt_plugin ${PACKAGE_INCLUDE_DEPENDENCIES})

pluginlib_export_plugin_description_file(moveit_core rrt_plugins.xml)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
# target_compile_definitions(rrt_plugin PRIVATE "RRT_PLUGIN_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS rrt_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  rrt_plugin
)
ament_export_targets(
  export_${PROJECT_NAME}
)

# demo
add_executable(rrt_demo src/rrt_demo.cpp)
target_include_directories(rrt_demo PUBLIC include)
ament_target_dependencies(rrt_demo ${PACKAGE_INCLUDE_DEPENDENCIES})


install(TARGETS rrt_demo
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES rrt_plugins.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
