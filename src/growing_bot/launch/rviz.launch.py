#!/usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = xacro.parse(open(xacro_file_path))
    xacro.process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process XACRO
    xacro_path = os.path.join(get_package_share_directory('growing_bot'), 'urdf', 'growing_bot.xacro')
    doc = get_xacro_to_doc(xacro_path, {"wheel_odom_topic": "odom"})

    robot_description = {
        'robot_description': ParameterValue(doc.toxml(), value_type=str)
    }

    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, robot_description],
    )

    # robot_state_publisher only publishes TF for fixed joints until /joint_states
    # is available. Standalone RViz needs this node; disable it when a simulator
    # already publishes joint states (e.g. js_gui:=false with Gazebo).
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('js_gui'))
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('js_gui'))
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('growing_bot'), 'rviz', 'entire_setup.rviz')]
    )

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time if true'),
        DeclareLaunchArgument('js_gui', default_value='true', description='Publish /joint_states via joint_state_publisher_gui (set false if Gazebo already publishes them)'),
         # Nodes
        GroupAction(
            actions=[joint_state_publisher_gui, joint_state_publisher],
        ),
        robot_state_publisher,
        rviz
    ])