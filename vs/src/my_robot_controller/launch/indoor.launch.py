import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # 1. PATHS
    pkg_controller = get_package_share_directory('my_robot_controller')
    pkg_description = get_package_share_directory('my_robot_description')

    urdf_file = os.path.join(pkg_description, 'urdf', 'robot.urdf')
    world_file = os.path.join(pkg_description, 'worlds', 'my_map.world')
    config_file = os.path.join(pkg_controller, 'config', 'safety_params.yaml')
    pcd_file = '/home/srikar/ros2_ws/vs/src/my_robot_description/maps/my_3d_map.pcd'
    rviz_config_path = os.path.join(pkg_controller, 'Rviz', 'my_robot_vs.rviz')
    
    # PATH TO YOUR WEB UI FOLDER (Verify this path matches yours!)
    web_ui_path = '/home/srikar/ros2_ws/web_ui'

    # 2. NODES
    robot_description = xacro.process_file(urdf_file).toxml()
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}, {'use_sim_time': True}],
        output='screen'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
        output='screen'
    )

    # 3. WEB SERVER & BRIDGE
    
    # A. Rosbridge (The Data Link)
    rosbridge = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml',
            'port:=9090', 'send_buffer_limit:=100000000'
        ],
        output='screen'
    )

    # B. Python Web Server (The Website) - AUTO START!
    web_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8000', '--directory', web_ui_path],
        output='screen'
    )

    # 4. NAVIGATION STACK
    dynamic_3d_map_manager = Node(
        package='my_robot_controller', executable='dynamic_3d_map_manager',
        parameters=[{'pcd_file_path': pcd_file, 'map_frame_id': 'map'}],
        output='screen'
    )

    map_to_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    obstacle_detector = Node(
        package='my_robot_controller', executable='obstacle_detector_3d',
        parameters=[config_file, {'use_sim_time': True}], output='screen',
    )

    costmap_2d = Node(
        package='my_robot_controller', executable='costmap_2d_from_3d',
        parameters=[{'use_sim_time': True}], output='screen'
    )

    a_star_planner = Node(
        package='my_robot_controller', executable='a_star_planner',
        parameters=[config_file, {'use_sim_time': True}], output='screen'
    )

    local_planner = Node(
        package='my_robot_controller', executable='local_planner',
        parameters=[config_file, {'use_sim_time': True}], output='screen'
    )

    camera_safety = Node(
        package='my_robot_vision', executable='camera_safety', output='screen'
    )
    
    safety_monitor = Node(
        package='my_robot_controller', executable='safety_monitor', output='screen'
    )

    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}], output='screen'
    )

    return LaunchDescription([
        map_to_odom, robot_state_publisher, gazebo, spawn_robot,
        rosbridge, web_server, # <--- Added web_server here
        dynamic_3d_map_manager, obstacle_detector, costmap_2d,
        a_star_planner, local_planner, camera_safety, safety_monitor, rviz
    ])
