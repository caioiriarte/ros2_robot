cmake_minimum_required(VERSION 3.5)

project(robot_1)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Install URDF, meshes, and other resources
install(DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_package()
