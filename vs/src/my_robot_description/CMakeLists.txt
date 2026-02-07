cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. We only need ament_cmake for a data package
find_package(ament_cmake REQUIRED)

# 2. INSTALL folders so ROS can find your URDF and World files
install(DIRECTORY
  urdf
  worlds
  maps
  DESTINATION share/${PROJECT_NAME}
)

# 3. No add_executable or ament_target_dependencies needed here!
# Unless you are writing C++ in THIS package, remove rclcpp lines.

ament_package()
