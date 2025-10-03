from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "dsr_zed_moveit_config"
    pkg_share = FindPackageShare(package_name)

    declared_arguments = [
        DeclareLaunchArgument(
            "mock_hardware",
            default_value="true",
            description="Mock /joint_states pub and robot_state_publisher?",
        ),
    ]

    def launch_setup(context, *args, **kwargs):
        # mock joint_states pub
        joint_state_publisher = Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            parameters=[
                {"rate": 25},
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("mock_hardware")),
        )

        # include rsp launch
        rsp_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([pkg_share, "launch", "rsp.launch.py"])]
            ),
            condition=IfCondition(LaunchConfiguration("mock_hardware")),
        )

        # include move_group launch
        move_group_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([pkg_share, "launch", "move_group.launch.py"])]
            ),
        )

        # include moveit_rviz launch
        rviz_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([pkg_share, "launch", "moveit_rviz.launch.py"])]
            ),
        )

        return [
            joint_state_publisher,
            rsp_launch,
            move_group_launch,
            rviz_launch,
        ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
