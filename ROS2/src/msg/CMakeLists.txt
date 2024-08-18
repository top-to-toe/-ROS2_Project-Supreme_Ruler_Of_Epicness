cmake_minimum_required(VERSION 3.8)
project(msg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# include directories
include_directories(include)

# excutable
add_executable(message_publisher_class src/message_publisher_class.cpp src/mpub.cpp)
ament_target_dependencies(message_publisher_class rclcpp std_msgs)

add_executable(message_subscriber_class src/message_subscriber_class.cpp src/msub.cpp)
ament_target_dependencies(message_subscriber_class rclcpp std_msgs)

add_executable(message2_subscriber_class src/message2_subscriber_class.cpp src/m2sub.cpp)
ament_target_dependencies(message2_subscriber_class rclcpp std_msgs)

add_executable(message_time_subscriber_class src/message_time_subscriber_class.cpp src/mtsub.cpp)
ament_target_dependencies(message_time_subscriber_class rclcpp std_msgs)

add_executable(time_publisher_class src/time_publisher_class.cpp src/tpub.cpp)
ament_target_dependencies(time_publisher_class rclcpp std_msgs)

# install
install(TARGETS
 message_publisher_class
 message_subscriber_class
 message2_subscriber_class
 message_time_subscriber_class
 time_publisher_class 
 DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  param
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
