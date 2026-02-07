cmake_minimum_required(VERSION 3.8)
project(my_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(std_srvs REQUIRED)
# PCL is special, we need the system libraries
find_package(PCL REQUIRED COMPONENTS common io filters)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ---------------------------------------------------
# 1. MAPPING NODES
# ---------------------------------------------------
add_executable(map_accumulator_3d src/map_accumulator_3d.cpp)
ament_target_dependencies(map_accumulator_3d rclcpp sensor_msgs std_srvs tf2 tf2_ros tf2_sensor_msgs pcl_conversions pcl_ros)
target_link_libraries(map_accumulator_3d ${PCL_LIBRARIES})

add_executable(map_loader_node src/map_loader_node.cpp)
ament_target_dependencies(map_loader_node rclcpp sensor_msgs pcl_conversions pcl_ros)
target_link_libraries(map_loader_node ${PCL_LIBRARIES})

add_executable(dynamic_3d_map_manager src/dynamic_3d_map_manager.cpp)
ament_target_dependencies(dynamic_3d_map_manager rclcpp sensor_msgs std_srvs pcl_conversions pcl_ros)
target_link_libraries(dynamic_3d_map_manager ${PCL_LIBRARIES})

add_executable(costmap_2d_from_3d src/costmap_2d_from_3d.cpp)
ament_target_dependencies(costmap_2d_from_3d rclcpp sensor_msgs nav_msgs pcl_conversions pcl_ros)
target_link_libraries(costmap_2d_from_3d ${PCL_LIBRARIES})

# ---------------------------------------------------
# 2. NAVIGATION NODES (A* & Local Planner)
# ---------------------------------------------------
add_executable(a_star_planner src/a_star_planner.cpp)
ament_target_dependencies(a_star_planner rclcpp nav_msgs geometry_msgs tf2 tf2_ros)

add_executable(local_planner src/local_planner.cpp)
ament_target_dependencies(local_planner rclcpp nav_msgs sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs pcl_conversions)
target_link_libraries(local_planner ${PCL_LIBRARIES})

# ---------------------------------------------------
# 3. SAFETY NODES
# ---------------------------------------------------
add_executable(obstacle_detector_3d src/obstacle_detector_3d.cpp)
ament_target_dependencies(obstacle_detector_3d rclcpp sensor_msgs std_msgs pcl_conversions pcl_ros)
target_link_libraries(obstacle_detector_3d ${PCL_LIBRARIES})

add_executable(safety_monitor src/safety_monitor.cpp)
ament_target_dependencies(safety_monitor rclcpp geometry_msgs std_msgs)

add_executable(lidar_listener src/lidar_listener.cpp)
ament_target_dependencies(lidar_listener rclcpp sensor_msgs)

# ---------------------------------------------------
# INSTALL RULES (Copy to Install Folder)
# ---------------------------------------------------
install(TARGETS
  map_accumulator_3d
  map_loader_node
  dynamic_3d_map_manager
  costmap_2d_from_3d
  a_star_planner
  local_planner
  obstacle_detector_3d
  safety_monitor
  lidar_listener
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  maps
  Rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
