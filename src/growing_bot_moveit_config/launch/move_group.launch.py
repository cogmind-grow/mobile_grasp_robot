import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import (
    DeclareBooleanLaunchArg,
    add_debuggable_node,
)

# 与 growing_bot gz_spawn / gz_ros2_control 的 robot_namespace 一致
ROBOT_NAMESPACE = "growing_bot"


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "growing_bot", package_name="growing_bot_moveit_config"
    ).to_moveit_configs()

    ld = LaunchDescription()

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    )
    ld.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    )
    ld.add_action(
        DeclareLaunchArgument(
            "capabilities",
            default_value=moveit_config.move_group_capabilities["capabilities"],
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "disable_capabilities",
            default_value=moveit_config.move_group_capabilities["disable_capabilities"],
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))
    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use /clock from Gazebo when running with simulation",
        )
    )

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": LaunchConfiguration("monitor_dynamics"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]

    # ros2_control / joint_state_broadcaster 在命名空间下发布
    remappings = [
        ("/joint_states", f"/{ROBOT_NAMESPACE}/joint_states"),
        (
            "/arm_controller/follow_joint_trajectory",
            f"/{ROBOT_NAMESPACE}/arm_controller/follow_joint_trajectory",
        ),
        (
            "/gripper_controller/follow_joint_trajectory",
            f"/{ROBOT_NAMESPACE}/gripper_controller/follow_joint_trajectory",
        ),
    ]

    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        commands_file=str(moveit_config.package_path / "launch" / "gdb_settings.gdb"),
        output="screen",
        parameters=move_group_params,
        remappings=remappings,
        extra_debug_args=["--debug"],
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
    )

    return ld
