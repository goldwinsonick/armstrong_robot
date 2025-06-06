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

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/oltf_data_acq.py
  scripts/effort_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(joint_pid_controller
  src/joint_pid_controller.cpp
)

install(TARGETS
  joint_pid_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
