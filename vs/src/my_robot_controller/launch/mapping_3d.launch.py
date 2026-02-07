import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # ===============================
    # 1. Package Paths & Files
    # ===============================
    pkg_controller = get_package_share_directory('my_robot_controller')
    pkg_description = get_package_share_directory('my_robot_description')

    urdf_file = os.path.join(pkg_description, 'urdf', 'robot.urdf')
    world_file = os.path.join(pkg_description, 'worlds', 'my_map.world')

    # ===============================
    # 2. Robot Description (URDF)
    # ===============================
    robot_description = xacro.process_file(urdf_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ],
        output='screen'
    )

    # ===============================
    # 3. Gazebo Simulation
    # ===============================
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

    # ===============================
    # 4. 3D Map Accumulator (The Mapper)
    # ===============================
    # This node listens to /livox/points and builds the global cloud
    map_accumulator = Node(
        package='my_robot_controller',
        executable='map_accumulator_3d',
        name='map_accumulator_3d',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )


    rviz = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ===============================
    # 6. Launch Execution
    # ===============================
    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_robot,
        map_accumulator,
        rviz
    ])
