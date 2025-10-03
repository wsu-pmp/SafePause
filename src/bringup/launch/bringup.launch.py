from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("perception_pkg"), "config", "example.yaml"]
            ),
            description="Topic config for perception_node, which requires one",
        ),
    ]

    def launch_setup(context, *args, **kwargs):
        launches = []

        launches.append(
            Node(
                package="perception_pkg",
                executable="perception_node",
                output="screen",
                parameters=[
                    {"config_file": LaunchConfiguration("config_file")},
                ],
                on_exit=Shutdown(),
            )
        )

        launches.append(
            Node(
                package="interrupt_ctrl",
                executable="interrupt_node",
                output="screen",
            )
        )

        return launches

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
