#!/usr/bin/env python3

from os.path import join


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
from launch.actions import AppendEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    growing_bot_path = get_package_share_directory('growing_bot')

    # Retrieve launch configuration arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    
    world_file = LaunchConfiguration("world_file", default = join(growing_bot_path, 'worlds', 'small_warehouse.sdf'))
    
    # Include the Gazebo launch file
    gazebo_share = get_package_share_directory("gazebo_ros")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gazebo_share, "launch", "gazebo.launch.py")),
        launch_arguments={
            'world': world_file,
            'gui': LaunchConfiguration('gui'),
            'verbose': LaunchConfiguration('verbose'),
        }.items(),
    )

    spawn_growing_bot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(growing_bot_path, "launch", "gazebo_spawn.launch.py")),
        launch_arguments={
            'ros_control_enabled': LaunchConfiguration('ros_control_enabled'),
            'arm_enabled': LaunchConfiguration('arm_enabled'),
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', join(growing_bot_path, 'rviz', 'entire_setup.rviz')]
    )

    return LaunchDescription([
        # Declare launch arguments
        
        AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=join(growing_bot_path, "models")),

        SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value="/usr/share/gazebo-11:" + join(growing_bot_path, "worlds")),
        DeclareLaunchArgument(
            'world_file',
            default_value=join(growing_bot_path, 'worlds', 'small_warehouse.sdf'),
            description='World file to load in Gazebo Classic'),
        DeclareLaunchArgument('world', default_value=world_file),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value = use_sim_time),
        DeclareLaunchArgument('arm_enabled', default_value='true'),
        DeclareLaunchArgument('ros_control_enabled', default_value='false'),
        gazebo, spawn_growing_bot_node, rviz
    ])
