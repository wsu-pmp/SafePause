import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration("config_file").perform(context)
    output_bag_dir = LaunchConfiguration("output_bag_dir").perform(context)

    # load config
    if not config_file:
        # use default config from package
        pkg_share = get_package_share_directory("rosbag2_multirecord")
        config_file = os.path.join(pkg_share, "config", "example.yaml")

        get_logger("launch").warning(
            "No 'config_file' launch argument provided, 'example.yaml' will be used."
        )

    with open(config_file, "r") as f:
        config = yaml.safe_load(f)

    # validate configuration
    if "topic_groups" not in config:
        raise ValueError("Config file must contain 'topic_groups' key")

    topic_groups = config["topic_groups"]
    if not topic_groups or len(topic_groups) == 0:
        raise ValueError("'topic_groups' must contain at least one group")

    # get output directory (use config if launch arg not provided)
    if not output_bag_dir:
        output_bag_dir = config.get("output_bag_dir", "./multirecord-out")

    qos_overrides = config["qos_overrides"]

    # create temporary directories for individual recorders
    temp_base_dir = tempfile.mkdtemp(prefix="rosbag2_multirecord_")

    # recorder nodes
    recorder_nodes = []
    recorder_namespaces = []

    for i, topics in enumerate(topic_groups):
        namespace = str(i)
        recorder_namespaces.append(namespace)

        recorder_output_dir = os.path.join(temp_base_dir, f"recorder_{i}")

        filtered_qos = {
            t: qos_overrides[t] for t in topics if qos_overrides and t in qos_overrides
        }

        recorder_node = Node(
            package="rosbag2_multirecord",
            executable="recorder",
            name=f"rosbag2_recorder_{namespace}",
            output="screen",
            arguments=[
                "--namespace",
                namespace,
            ],
            parameters=[
                {
                    "topics": topics,
                    "output_dir": recorder_output_dir,
                    "qos_overrides_yaml": yaml.dump(filtered_qos)
                    if filtered_qos
                    else "",
                }
            ],
        )
        recorder_nodes.append(recorder_node)

    # coordinator node
    coordinator_node = Node(
        package="rosbag2_multirecord",
        executable="coordinator",
        name="multirecord_coordinator",
        output="screen",
        parameters=[
            {
                "output_bag_dir": output_bag_dir,
                "recorder_namespaces": recorder_namespaces,
            }
        ],
    )

    return recorder_nodes + [coordinator_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value="",
            ),
            DeclareLaunchArgument(
                "output_bag_dir",
                default_value="",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
