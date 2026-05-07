cmake_minimum_required(VERSION 3.8)
project(canvas_pose_detector)

# ── Compiler settings ────────────────────────────────────────────────────────
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

# ── Find dependencies ────────────────────────────────────────────────────────
find_package(ament_cmake        REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(rclpy              REQUIRED)
find_package(sensor_msgs        REQUIRED)
find_package(geometry_msgs      REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros            REQUIRED)
find_package(tf2_geometry_msgs  REQUIRED)
find_package(message_filters    REQUIRED)
find_package(cv_bridge          REQUIRED)
find_package(OpenCV             REQUIRED)
find_package(Eigen3             REQUIRED)
find_package(std_msgs           REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ── C++ executable: canvas_pose_detector ─────────────────────────────────────
add_executable(canvas_pose_detector
  src/canvas_pose_detector.cpp
)

target_include_directories(canvas_pose_detector PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(canvas_pose_detector
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  message_filters
  cv_bridge
  std_msgs
  builtin_interfaces
)

target_link_libraries(canvas_pose_detector
  ${OpenCV_LIBRARIES}
  Eigen3::Eigen
)

# ── Install C++ executable ────────────────────────────────────────────────────
install(TARGETS canvas_pose_detector
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install Python scripts as executables ────────────────────────────────────
# One install(PROGRAMS) call per script so we can RENAME each one to drop
# the .py extension -- ROS2 node lookup requires the executable name to
# match the name in the launch file exactly (no extension).
install(PROGRAMS src/canvas_pose_visualiser.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME canvas_pose_visualiser
)
install(PROGRAMS src/canvas_pose_injector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME canvas_pose_injector
)
install(PROGRAMS src/canvas_pose_follower.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME canvas_pose_follower
)
install(PROGRAMS src/canvas_stroke_corrector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME canvas_stroke_corrector
)
install(PROGRAMS src/canvas_correction_visualiser.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME canvas_correction_visualiser
)
install(PROGRAMS src/canvas_depth_visualiser.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME canvas_depth_visualiser
)

install(PROGRAMS
  src/canvas_tf_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME canvas_tf_bridge
)


# ── Install launch files ──────────────────────────────────────────────────────
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# ── Install config directory (YAML params, RViz configs etc.) ─────────────────
# Create the directory first if it doesn't exist yet:
#   mkdir -p ~/ros2_ws/src/canvas_pose_detector/config
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  config/rviz_tf_validation.rviz
  DESTINATION share/${PROJECT_NAME}/config
)

install(FILES
  config/rviz_full.rviz
  DESTINATION share/${PROJECT_NAME}/config
)
# ── Ament package finalisation ────────────────────────────────────────────────
ament_package()