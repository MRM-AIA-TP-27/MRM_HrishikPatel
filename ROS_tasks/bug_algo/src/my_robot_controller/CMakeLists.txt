cmake_minimum_required(VERSION 3.8)
project(my_robot_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 2. Include PCL Headers
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# --- 3. EXECUTABLE 1: GPS Navigator (State Machine) ---
add_executable(gps_navigator src/gps_navigator.cpp)
target_link_libraries(gps_navigator ${PCL_LIBRARIES})
ament_target_dependencies(gps_navigator
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs

   pcl_conversions
)

# --- 4. EXECUTABLE 2: Obstacle Detector (Clustering) ---
 add_executable(obstacle_detector src/obstacle_detector.cpp)
 target_link_libraries(obstacle_detector ${PCL_LIBRARIES})
 ament_target_dependencies(obstacle_detector
   rclcpp
   sensor_msgs
   pcl_conversions
 )

# 5. Install Rules
# This makes the binaries available to 'ros2 run'
install(TARGETS
  gps_navigator
  obstacle_detector
  DESTINATION lib/${PROJECT_NAME}
)

# This makes your assets available to 'ros2 launch'
install(DIRECTORY
  launch
  urdf
  rviz
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# 6. Package Export
ament_package()