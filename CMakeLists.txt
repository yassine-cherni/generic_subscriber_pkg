cmake_minimum_required(VERSION 3.8)
project(generic_subscriber_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add executables
add_executable(generic_subscriber src/generic_subscriber.cpp)
add_executable(string_publisher src/string_publisher.cpp)
add_executable(int_publisher src/int_publisher.cpp)
add_executable(float_publisher src/float_publisher.cpp)

# Specify dependencies
ament_target_dependencies(generic_subscriber rclcpp std_msgs)
ament_target_dependencies(string_publisher rclcpp std_msgs)
ament_target_dependencies(int_publisher rclcpp std_msgs)
ament_target_dependencies(float_publisher rclcpp std_msgs)

# Install targets
install(TARGETS
  generic_subscriber
  string_publisher
  int_publisher
  float_publisher
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()