cmake_minimum_required(VERSION 3.8)
project(armstrong_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/oltf_data_acq.py
  scripts/effort_teleop.py
  scripts/joint2_target_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(joint_pid_controller
  src/joint_pid_controller.cpp
)
ament_target_dependencies(joint_pid_controller rclcpp sensor_msgs std_msgs)

install(TARGETS
  joint_pid_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
