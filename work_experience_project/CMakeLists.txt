cmake_minimum_required(VERSION 3.8)
project(work_experience_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)  # 컴파일 시 경고를 강화합니다.
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)  # 네비게이션 관련 메시지 사용
find_package(tf2 REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories for OpenCV
include_directories(include ${OpenCV_INCLUDE_DIRS})

# Add executables for each node
add_executable(waypoints_navigation_node src/waypoints_navigation_node.cpp)
ament_target_dependencies(waypoints_navigation_node 
  rclcpp 
  nav2_msgs 
  rclcpp_action 
  tf2 
  geometry_msgs
  tf2_geometry_msgs 
  tf2_ros  
  std_srvs
)

add_executable(main_control_node src/main_control_node.cpp)
ament_target_dependencies(main_control_node 
  rclcpp 
  geometry_msgs 
  std_srvs  
  std_msgs  
  tf2
  tf2_geometry_msgs
  sensor_msgs
  cv_bridge  
  OpenCV     
)

add_executable(waypoints_action_client src/waypoints_action_client.cpp)
ament_target_dependencies(waypoints_action_client 
  rclcpp 
  geometry_msgs 
  std_srvs  
  tf2 
  tf2_geometry_msgs
  tf2_ros
  nav2_msgs
)

# Install the executables
install(TARGETS
  waypoints_navigation_node
  main_control_node
  waypoints_action_client
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts if any
install(
  PROGRAMS
  # Add Python scripts here if necessary
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

# Set up for testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
