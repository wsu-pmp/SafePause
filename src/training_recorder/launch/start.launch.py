from typing import Dict

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_passthrough_args(
    launch_configs: Dict[str, str],
) -> Dict[str, Dict[str, str]]:
    passthrough_args: Dict[str, Dict[str, str]] = dict()

    for argk, argv in launch_configs.items():
        prefix, delim, key = argk.partition(".")
        if delim:
            passthrough_args.setdefault(prefix.lower(), dict()).update({key: argv})

    return passthrough_args


def generate_launch_description():
    declared_arguments = []

    def launch_setup(context, *args, **kwargs):
        passthrough_args = generate_passthrough_args(context.launch_configurations)

        # dsr_zed_moveit_config
        moveit_prefix = "moveit"
        moveit_arg_defaults = {}
        moveit_arg_overrides = {}
        moveit_combined_launch_args = {
            **moveit_arg_defaults,
            **passthrough_args.get(moveit_prefix, dict()),
            **moveit_arg_overrides,
        }

        dsr_zed_moveit_config = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("dsr_zed_moveit_config"),
                            "launch",
                            "start.launch.py",
                        ]
                    )
                ]
            ),
            launch_arguments=moveit_combined_launch_args.items(),
        )

        # recording_controller
        recording_controller_prefix = "recording_controller"
        recording_controller_arg_defaults = {}
        recording_controller_arg_overrides = {}
        recording_controller_combined_launch_args = {
            **recording_controller_arg_defaults,
            **passthrough_args.get(recording_controller_prefix, dict()),
            **recording_controller_arg_overrides,
        }

        recording_controller = Node(
            package="training_recorder",
            executable="recording_controller",
            output="screen",
            parameters=[
                {k: v} for k, v in recording_controller_combined_launch_args.items()
            ],
        )

        return [dsr_zed_moveit_config, recording_controller]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
