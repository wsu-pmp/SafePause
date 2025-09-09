from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = []

    def launch_setup(context, *args, **kwargs):
        launches = []

        launches.append(Node(package="perception_pkg", executable="perception_node"))

        launches.append(Node(package="interrupt_ctrl", executable="interrupt_node"))

        return launches

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
