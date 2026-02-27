cmake_minimum_required(VERSION 3.8)
project(scanner_routine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(scanner_action_interfaces REQUIRED)
find_package(scanner_assembler REQUIRED)
find_package(tf2 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(scanner_action_server src/scanner_action_server.cpp)
ament_target_dependencies(scanner_action_server
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  scanner_action_interfaces
  scanner_assembler
  tf2
  pcl_ros
  pcl_conversions
  visualization_msgs
)
target_link_libraries(scanner_action_server ${PCL_LIBRARIES})

add_executable(scanner_action_client src/scanner_action_client.cpp)
ament_target_dependencies(scanner_action_client
  rclcpp
  rclcpp_action
  scanner_action_interfaces
)

install(TARGETS
  scanner_action_server
  scanner_action_client
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

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
