import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 一键启动整个抓取系统，等价于原来的 4 个终端：
    #   终端1：ros2 launch growing_bot gz.launch.py ros_control_enabled:=true
    #   终端2：ros2 launch growing_bot_moveit_config move_group.launch.py use_sim_time:=true
    #   终端3：ros2 launch growing_bot nav2.launch.py
    #   终端4：ros2 launch detector pick.launch.py
    #
    # 时序说明：仿真先启动，延时 20s 等 Gazebo/机器人/传感器就绪后，
    # 再启动 MoveIt / Nav2 / 检测+状态机（TimerAction 计时用墙钟，不受 use_sim_time 影响）
    growing_bot_share = get_package_share_directory('growing_bot')
    moveit_share = get_package_share_directory('growing_bot_moveit_config')
    detector_share = get_package_share_directory('detector')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # 终端1：Gazebo 仿真（gz_sim + 机器人 spawn + RViz）
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(growing_bot_share, 'launch', 'gz.launch.py')),
        launch_arguments={
            'ros_control_enabled': 'true',  # ros2_control 控制器
            'use_sim_time': use_sim_time,
        }.items()
    )

    # 终端2：MoveIt move_group（规划 + 轨迹执行）
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_share, 'launch', 'move_group.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 终端3：Nav2 导航（map_server + AMCL + 控制器）
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(growing_bot_share, 'launch', 'nav2.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 终端4：YOLO 检测 + 抓取状态机（复用 pick.launch.py）
    pick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(detector_share, 'launch', 'pick.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 仿真启动 20s 后，再启动 MoveIt / Nav2 / 检测+状态机
    delayed_start = TimerAction(
        period=20.0,
        actions=[moveit_launch, nav2_launch, pick_launch],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='使用仿真时钟（/clock）'),
        gz_launch,
        delayed_start,
    ])
