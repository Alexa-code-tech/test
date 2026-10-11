import os
from ament_index_python.packages import get_package_share_directory as pkg
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = pkg('ab_nav')
    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(share, 'urdf', 'ab_bot.urdf.xacro')]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': os.path.join(share, 'worlds', 'ab.world'),
                          'gui': LaunchConfiguration('gui')}.items())

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={'slam_params_file': os.path.join(share, 'config', 'slam_params.yaml'),
                          'use_sim_time': 'true'}.items())

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={'params_file': os.path.join(share, 'config', 'nav2_params.yaml'),
                          'use_sim_time': 'true', 'autostart': 'true'}.items())

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='окно Gazebo'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('mission', default_value='true', description='автозапуск миссии A->B'),
        gazebo,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}]),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-topic', 'robot_description', '-entity', 'ab_bot',
                        '-x', '0', '-y', '0', '-z', '0.01', '-Y', '0']),
        slam,
        nav2,
        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', os.path.join(pkg('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(package='ab_nav', executable='gps_waypoint_nav', output='screen',
             parameters=[{'use_sim_time': True,
                          'waypoints_file': os.path.join(share, 'config', 'waypoints.yaml')}],
             condition=IfCondition(LaunchConfiguration('mission'))),
    ])
