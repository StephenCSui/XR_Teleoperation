cmake_minimum_required(VERSION 3.8)
project(unity_authority_filter_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)

find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)

find_package(Eigen3 REQUIRED)

add_executable(unity_authority_filter src/unity_authority_filter.cpp)
ament_target_dependencies(unity_authority_filter
  rclcpp
  geometry_msgs
  sensor_msgs
  moveit_core
  moveit_ros_planning
)
target_include_directories(unity_authority_filter SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIRS})

add_executable(unity_goal_executor src/unity_goal_executor.cpp)
ament_target_dependencies(unity_goal_executor
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  control_msgs
  moveit_core
  moveit_ros_planning
)
target_include_directories(unity_goal_executor SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIRS})

install(TARGETS
  unity_authority_filter
  unity_goal_executor
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()