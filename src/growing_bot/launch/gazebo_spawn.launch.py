#!/usr/bin/env python3

from os.path import join
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch.substitutions import PythonExpression

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = xacro.parse(open(xacro_file_path))
    xacro.process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():
    growing_bot_path = get_package_share_directory('growing_bot')
    
    # Retrieve launch configuration arguments
    position_x = LaunchConfiguration("position_x")
    position_y = LaunchConfiguration("position_y")
    orientation_yaw = LaunchConfiguration("orientation_yaw")
    camera_enabled = LaunchConfiguration("camera_enabled", default=True)
    stereo_camera_enabled = LaunchConfiguration("stereo_camera_enabled", default=False)
    two_d_lidar_enabled = LaunchConfiguration("two_d_lidar_enabled", default=True)
    arm_enabled = LaunchConfiguration("arm_enabled", default=True)
    ros_control_enabled = LaunchConfiguration("ros_control_enabled", default=False)
    odometry_source = LaunchConfiguration("odometry_source", default="world")
    robot_namespace = LaunchConfiguration("robot_namespace", default='growing_bot')
    
    # Path to the Xacro file
    xacro_path = join(growing_bot_path, 'urdf', 'growing_bot.xacro')

    # Launch the robot_state_publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(
                Command([
                    'xacro ', xacro_path,
                    ' camera_enabled:=', camera_enabled,
                    ' stereo_camera_enabled:=', stereo_camera_enabled,
                    ' two_d_lidar_enabled:=', two_d_lidar_enabled,
                    ' arm_enabled:=', arm_enabled,
                    ' sim_gazebo:=', 'true',
                    ' ros_control_enabled:=', ros_control_enabled,
                    ' odometry_source:=', odometry_source,
                    ' robot_namespace:=', robot_namespace,
                ]),
                value_type=str,
            ),
        }],
        remappings=[
            ('/joint_states', PythonExpression(['"', robot_namespace, '/joint_states"'])),
        ]
    )

    # Launch the spawn_entity node to spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', "/robot_description",
            '-entity', PythonExpression(['"', robot_namespace, '_robot"']),
            '-z', "0.28",
            '-x', position_x,
            '-y', position_y,
            '-Y', orientation_yaw
        ]
    )

    controller_manager = PythonExpression(['"', robot_namespace, '/controller_manager"'])
    arm_ros_control = PythonExpression([
        "'", ros_control_enabled, "' == 'true' and '", arm_enabled, "' == 'true'"
    ])

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        condition=IfCondition(ros_control_enabled),
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', controller_manager,
            '--controller-manager-timeout', '60',
        ],
        parameters=[{'use_sim_time': True}],
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        condition=IfCondition(ros_control_enabled),
        arguments=[
            'diff_drive_controller',
            '--controller-manager', controller_manager,
            '--controller-manager-timeout', '60',
        ],
        parameters=[{'use_sim_time': True}],
    )

    arm_controllers_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        condition=IfCondition(arm_ros_control),
        arguments=[
            'arm_controller',
            'gripper_controller',
            '--controller-manager', controller_manager,
            '--controller-manager-timeout', '60',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # Controllers after the model exists in Gazebo
    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    load_diff_drive = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    load_arm_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diff_drive_spawner,
            on_exit=[arm_controllers_spawner],
        )
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument("camera_enabled", default_value = camera_enabled),
        DeclareLaunchArgument("stereo_camera_enabled", default_value = stereo_camera_enabled),
        DeclareLaunchArgument("two_d_lidar_enabled", default_value = two_d_lidar_enabled),
        DeclareLaunchArgument("arm_enabled", default_value = arm_enabled),
        DeclareLaunchArgument("ros_control_enabled", default_value="false"),
        DeclareLaunchArgument("position_x", default_value="0.0"),
        DeclareLaunchArgument("position_y", default_value="0.0"),
        DeclareLaunchArgument("orientation_yaw", default_value="0.0"),
        DeclareLaunchArgument("odometry_source", default_value = odometry_source),
        DeclareLaunchArgument("robot_namespace", default_value = robot_namespace),
        robot_state_publisher,
        spawn_entity,
        load_joint_state_broadcaster,
        load_diff_drive,
        load_arm_controllers,
    ])
