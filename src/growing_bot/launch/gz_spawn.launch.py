#!/usr/bin/python3

from os.path import join
from xacro import parse, process_doc

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch.substitutions import PythonExpression

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = parse(open(xacro_file_path))
    process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():
   
    growing_bot_path = get_package_share_directory("growing_bot")
    position_x = LaunchConfiguration("position_x")
    position_y = LaunchConfiguration("position_y")
    orientation_yaw = LaunchConfiguration("orientation_yaw")
    camera_enabled = LaunchConfiguration("camera_enabled")
    stereo_camera_enabled = LaunchConfiguration("stereo_camera_enabled")
    two_d_lidar_enabled = LaunchConfiguration("two_d_lidar_enabled")
    arm_enabled = LaunchConfiguration("arm_enabled")
    detachable_joint = LaunchConfiguration("detachable_joint")
    ros_control_enabled = LaunchConfiguration("ros_control_enabled")
    odometry_source = LaunchConfiguration("odometry_source")
    robot_namespace = LaunchConfiguration("robot_namespace")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(
                Command([
                    'xacro ', join(growing_bot_path, 'urdf/growing_bot.xacro'),
                    ' camera_enabled:=', camera_enabled,
                    ' stereo_camera_enabled:=', stereo_camera_enabled,
                    ' two_d_lidar_enabled:=', two_d_lidar_enabled,
                    ' arm_enabled:=', arm_enabled,
                    ' detachable_joint:=', detachable_joint,
                    ' odometry_source:=', odometry_source,
                    ' ros_control_enabled:=', ros_control_enabled,
                    ' robot_namespace:=', robot_namespace,
                    ' sim_gz:=', 'true',
                ]),
                value_type=str,
            ),
        }],
        remappings=[
            ('/joint_states', PythonExpression(['"', robot_namespace, '/joint_states"'])),
        ]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "growing_bot",
            "-allow_renaming", "true",
            "-z", "0.28",
            "-x", position_x,
            "-y", position_y,
            "-Y", orientation_yaw
        ]
    )

    bridge_remappings = [
        ('/world/default/model/growing_bot/joint_state', 'growing_bot/joint_states'),
        ('/odom', 'growing_bot/odom'),
        ('/scan', 'growing_bot/scan'),
        ('/kinect_camera', 'growing_bot/kinect_camera'),
        ('/rgb_camera', 'growing_bot/rgb_camera'),
        ('/stereo_camera/left/image_raw', 'growing_bot/stereo_camera/left/image_raw'),
        ('/stereo_camera/right/image_raw', 'growing_bot/stereo_camera/right/image_raw'),
        ('/imu', 'growing_bot/imu'),
        ('/cmd_vel', 'growing_bot/cmd_vel'),
        ('kinect_camera/camera_info', 'growing_bot/kinect_camera/camera_info'),
        ('rgb_camera/camera_info', 'growing_bot/rgb_camera/camera_info'),
        ('stereo_camera/left/camera_info', 'growing_bot/stereo_camera/left/camera_info'),
        ('stereo_camera/right/camera_info', 'growing_bot/stereo_camera/right/camera_info'),
        ('/kinect_camera/points', 'growing_bot/kinect_camera/points'),
        ('/arm/joint1/cmd_pos', 'growing_bot/arm/joint1/cmd_pos'),
        ('/arm/joint2/cmd_pos', 'growing_bot/arm/joint2/cmd_pos'),
        ('/arm/joint3/cmd_pos', 'growing_bot/arm/joint3/cmd_pos'),
        ('/arm/joint4/cmd_pos', 'growing_bot/arm/joint4/cmd_pos'),
        ('/arm/joint5/cmd_pos', 'growing_bot/arm/joint5/cmd_pos'),
        ('/arm/joint6/cmd_pos', 'growing_bot/arm/joint6/cmd_pos'),
        ('/gripper/left/cmd_pos', 'growing_bot/gripper/left/cmd_pos'),
        ('/gripper/right/cmd_pos', 'growing_bot/gripper/right/cmd_pos'),
    ]

    # Without ros2_control: DiffDrive + native arm cmd_pos via bridge
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        condition=UnlessCondition(ros_control_enabled),
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/kinect_camera@sensor_msgs/msg/Image[gz.msgs.Image",
            "/rgb_camera@sensor_msgs/msg/Image[gz.msgs.Image",
            "/stereo_camera/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "stereo_camera/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "kinect_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "rgb_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/stereo_camera/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/stereo_camera/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/kinect_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/world/default/model/growing_bot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/arm/joint1/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            "/arm/joint2/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            "/arm/joint3/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            "/arm/joint4/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            "/arm/joint5/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            "/arm/joint6/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            "/gripper/left/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            "/gripper/right/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
        ],
        remappings=bridge_remappings,
    )

    # With ros2_control: sensors/clock only; cmd_vel/odom/joint_states from controllers
    gz_ros2_bridge_ros_control = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        condition=IfCondition(ros_control_enabled),
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/kinect_camera@sensor_msgs/msg/Image[gz.msgs.Image",
            "/rgb_camera@sensor_msgs/msg/Image[gz.msgs.Image",
            "/stereo_camera/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "stereo_camera/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "kinect_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "rgb_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/stereo_camera/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/stereo_camera/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/kinect_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        ],
        remappings=[
            ('/scan', 'growing_bot/scan'),
            ('/kinect_camera', 'growing_bot/kinect_camera'),
            ('/rgb_camera', 'growing_bot/rgb_camera'),
            ('/stereo_camera/left/image_raw', 'growing_bot/stereo_camera/left/image_raw'),
            ('/stereo_camera/right/image_raw', 'growing_bot/stereo_camera/right/image_raw'),
            ('/imu', 'growing_bot/imu'),
            ('kinect_camera/camera_info', 'growing_bot/kinect_camera/camera_info'),
            ('rgb_camera/camera_info', 'growing_bot/rgb_camera/camera_info'),
            ('stereo_camera/left/camera_info', 'growing_bot/stereo_camera/left/camera_info'),
            ('stereo_camera/right/camera_info', 'growing_bot/stereo_camera/right/camera_info'),
            ('/kinect_camera/points', 'growing_bot/kinect_camera/points'),
        ]
    )

    # DetachableJoint bridges (only when detachable_joint:=true)
    gz_detachable_joint_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        condition=IfCondition(detachable_joint),
        arguments=[
            "/growing_bot/bottle/attach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/growing_bot/bottle/detach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/growing_bot/bottle/attach_state@std_msgs/msg/String[gz.msgs.StringMsg",
        ],
    )

    # Bottle is spawned after the robot so DetachableJoint cannot weld it on at t=0
    # (Harmonic starts attached by default and would tip the mobile base).
    def make_spawn_bottle():
        return Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-file", join(growing_bot_path, "models", "bottle", "bottle.sdf"),
                "-name", "bottle",
                "-x", "1.55",
                "-y", "0.0",
                "-z", "0.0",
            ],
            output="screen",
        )

    # detachable_joint:=true → detach first, then spawn bottle
    spawn_bottle_with_detach = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[
                TimerAction(
                    period=30.0,
                    actions=[
                        ExecuteProcess(
                            cmd=[
                                'ros2', 'topic', 'pub', '--once',
                                '/growing_bot/bottle/detach',
                                'std_msgs/msg/Empty', '{}',
                            ],
                            output='screen',
                        ),
                    ],
                ),
                TimerAction(
                    period=15.0,
                    actions=[make_spawn_bottle()],
                ),
            ],
        ),
        condition=IfCondition(detachable_joint),
    )

    controller_manager = PythonExpression(['"', robot_namespace, '/controller_manager"'])
    controller_odom_remap = PythonExpression([
        "'--ros-args --remap /", robot_namespace,
        "/diff_drive_controller/odom:=/", robot_namespace, "/odom'"
    ])
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
            '--controller-ros-args', controller_odom_remap,
        ],
        parameters=[{'use_sim_time': True}],
    )

    cmd_vel_stamper = Node(
        package='growing_bot',
        executable='cmd_vel_stamper',
        namespace=robot_namespace,
        output='screen',
        condition=IfCondition(ros_control_enabled),
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

    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
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
        DeclareLaunchArgument("camera_enabled", default_value="true"),
        DeclareLaunchArgument("stereo_camera_enabled", default_value="false"),
        DeclareLaunchArgument("two_d_lidar_enabled", default_value="true"),
        DeclareLaunchArgument("arm_enabled", default_value="true"),
        DeclareLaunchArgument("detachable_joint", default_value="true"),
        DeclareLaunchArgument("ros_control_enabled", default_value="false"),
        DeclareLaunchArgument("position_x", default_value="0.0"),
        DeclareLaunchArgument("position_y", default_value="0.0"),
        DeclareLaunchArgument("orientation_yaw", default_value="0.0"),
        DeclareLaunchArgument("odometry_source", default_value="world"),
        DeclareLaunchArgument("robot_namespace", default_value="growing_bot"),
        robot_state_publisher,
        gz_spawn_entity,
        gz_ros2_bridge,
        gz_ros2_bridge_ros_control,
        gz_detachable_joint_bridge,
        spawn_bottle_with_detach,
        load_joint_state_broadcaster,
        load_diff_drive,
        cmd_vel_stamper,
        load_arm_controllers,
    ])
