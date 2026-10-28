import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav2_dir = get_package_share_directory('nav2_bringup')
    pkg_growing_bot = get_package_share_directory('growing_bot')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    autostart = LaunchConfiguration('autostart', default='True')

    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'map': os.path.join(pkg_growing_bot, 'config', 'map.yaml'),
            'params_file': os.path.join(pkg_growing_bot, 'config', 'nav2_params.yaml'),
            'package_path': pkg_growing_bot,
        }.items()
    )

    rviz_launch_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-d' + os.path.join(
                get_package_share_directory('nav2_bringup'),
                'rviz',
                'nav2_default_view.rviz'
            )
        ]
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': os.path.join(pkg_growing_bot, 'config', 'map.yaml')}],
    )

    remapper_node = Node(
        package='growing_bot',
        executable='remapper.py',
        name='remapper',
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(nav2_launch_cmd)
    ld.add_action(rviz_launch_cmd)

    return ld

