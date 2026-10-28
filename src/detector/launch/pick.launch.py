from glob import glob

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 一条命令启动"检测 + 抓取"，等价于原来的终端4和终端5：
    #   ros2 run detector bottle_localizer_node --ros-args -p use_sim_time:=true
    #   ros2 run growing_bot pick_state_machine --ros-args -p use_sim_time:=true
    use_sim_time = LaunchConfiguration('use_sim_time')
    model_path = LaunchConfiguration('model_path')

    # 终端4：YOLO 检测瓶子，发布 /growing_bot/bottle/position
    bottle_localizer_node = Node(
        package='detector',
        executable='bottle_localizer_node',
        name='bottle_localizer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'model_path': model_path,
        }],
    )

    # 终端5：抓取状态机（C++ 版，growing_bot 包），订阅瓶子位置并控制底盘+机械臂
    pick_state_machine_node = Node(
        package='growing_bot',
        executable='pick_state_machine',
        name='pick_state_machine',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='使用仿真时钟（/clock）'),
        DeclareLaunchArgument(
            'model_path', default_value='yolo11n.pt',
            description='YOLO 模型路径'),
        bottle_localizer_node,
        pick_state_machine_node,
    ])
