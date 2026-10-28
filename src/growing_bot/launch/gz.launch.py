#!/usr/bin/python3

from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch.actions import AppendEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    growing_bot_path = get_package_share_directory("growing_bot")
    default_world_file = join(growing_bot_path, "worlds", "small_warehouse.sdf")

    use_sim_time = LaunchConfiguration("use_sim_time")
    world_file = LaunchConfiguration("world_file")

    try:
        gz_sim_share = get_package_share_directory("ros_gz_sim")
    except PackageNotFoundError as exc:
        raise RuntimeError("未找到 ros_gz_sim, 请安装") from exc

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args" : PythonExpression(["'", world_file, " -r'"])

        }.items()
    )

    spawn_growing_bot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(growing_bot_path, "launch", "gz_spawn.launch.py")),
        launch_arguments={
            'ros_control_enabled': LaunchConfiguration('ros_control_enabled'),
            'arm_enabled': LaunchConfiguration('arm_enabled'),
            'detachable_joint': LaunchConfiguration('detachable_joint'),
        }.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', join(growing_bot_path, 'rviz', 'entire_setup.rviz')]
    )

    return LaunchDescription([

        AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=join(growing_bot_path, "worlds")),

        AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=join(growing_bot_path, "models")),

        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("world_file", default_value=default_world_file),
        DeclareLaunchArgument("arm_enabled", default_value="true"),
        DeclareLaunchArgument("detachable_joint", default_value="true"),
        DeclareLaunchArgument("ros_control_enabled", default_value="false"),

        gz_sim, spawn_growing_bot_node, rviz
    ])