cmake_minimum_required(VERSION 3.8)
project(my_robotic_arm)  

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies needed for this package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)

# Install all simulation assets in one clean block
install(DIRECTORY
  config
  launch
  meshes
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


find_package(moveit_servo REQUIRED)

find_package(moveit_ros_planning REQUIRED) # <--- Added
find_package(tf2_ros REQUIRED)             # <--- Added

add_executable(simple_servo_node src/simple_servo.cpp)
ament_target_dependencies(simple_servo_node 
  rclcpp 
  moveit_servo 
  moveit_ros_planning  # <--- Added
  tf2_ros              # <--- Added
)

install(TARGETS
  simple_servo_node
  DESTINATION lib/${PROJECT_NAME}
)