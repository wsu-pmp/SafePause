from launch import LaunchDescription
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # process URDF with xacro
    urdf_file = PathJoinSubstitution(
        [
            FindPackageShare("a0509_description"),
            "urdf",
            "a0509.urdf.xacro",
        ]
    )
    robot_description = ParameterValue(Command(["xacro ", urdf_file]), value_type=str)

    # publisher for /robot_description
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # static /joint_states publisher (all joints zero)
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
    )

    # rviz
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("a0509_description"),
            "rviz",
            "a0509.rviz",
        ]
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    # trigger shutdown when rviz is closed
    close_rviz_handler = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=rviz, on_exit=[Shutdown()])
    )

    return LaunchDescription(
        [robot_state_publisher, joint_state_publisher, rviz, close_rviz_handler]
    )
