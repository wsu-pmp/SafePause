from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("object_approach"), "config", "params.yaml"]
        ),
        description="Path to parameter file",
    )

    approach_node = Node(
        package="object_approach",
        executable="object_approach_node",
        name="object_approach_node",
        output="screen",
        parameters=[LaunchConfiguration("config_file")],
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            config_file_arg,
            approach_node,
        ]
    )
