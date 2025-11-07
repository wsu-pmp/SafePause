# Modified from
# https://raw.githubusercontent.com/DoosanRobotics/doosan-robot2/refs/heads/humble/dsr_moveit2/dsr_moveit_config_a0509/launch/start.launch.py

import os

from ament_index_python.packages import get_package_share_directory

# from dsr_bringup2.utils import read_update_rate
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Moveit2
from moveit_configs_utils import MoveItConfigsBuilder


def rviz_node_function(context):
    """Evaluate the model value at launch time, find the package path, and then execute the launch file"""
    # CHANGE: substitue package name and remove dynamic model_value
    # model_value = LaunchConfiguration('model').perform(context)

    # model_value_str = f"{model_value}"
    package_name_str = "dsr_zed_moveit_config"

    # Get the package path using FindPackageShare
    package_path_str = FindPackageShare(package_name_str).perform(context)

    print("Package name:", package_name_str)
    print("Package path:", package_path_str)

    # CHANGE: use default dsr_zed_moveit_config MoveItConfigsBuilder
    moveit_config = MoveItConfigsBuilder(
        "a0509", package_name=package_name_str
    ).to_moveit_configs()

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        # namespace=LaunchConfiguration('name'),
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    # RViz
    # CHANGE: rviz config share dir launch -> rviz
    rviz_base = os.path.join(get_package_share_directory(package_name_str), "rviz")
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")

    return [
        run_move_group_node,
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            # namespace=LaunchConfiguration('name'),
            output="log",
            arguments=["-d", rviz_full_config],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
        ),
    ]


def generate_launch_description():
    # CHANGE: remove model launch arg
    #         remove color launch arg
    #         add controller_log_level launch arg
    #         add launch_zed launch arg
    ARGUMENTS = [
        DeclareLaunchArgument("name", default_value="", description="NAME_SPACE"),
        DeclareLaunchArgument(
            "host", default_value="127.0.0.1", description="ROBOT_IP"
        ),
        DeclareLaunchArgument("port", default_value="12345", description="ROBOT_PORT"),
        DeclareLaunchArgument(
            "mode", default_value="virtual", description="OPERATION MODE"
        ),
        # DeclareLaunchArgument('model', default_value = 'm0617',     description = 'ROBOT_MODEL'    ),
        # DeclareLaunchArgument('color', default_value = 'white',     description = 'ROBOT_COLOR'    ),
        DeclareLaunchArgument("gui", default_value="false", description="Start RViz2"),
        DeclareLaunchArgument(
            "gz", default_value="false", description="USE GAZEBO SIM"
        ),
        DeclareLaunchArgument(
            "rt_host", default_value="192.168.137.50", description="ROBOT_RT_IP"
        ),
        DeclareLaunchArgument(
            "controller_log_level",
            default_value="warn",
            description="ros2_control_node log-level",
        ),
        DeclareLaunchArgument(
            "launch_zed",
            default_value="true",
            description="Launch zed_wrapper (zed2i)?",
        ),
    ]

    # update_rate = str(read_update_rate())  # get update_rate from yaml

    # CHANGE: substitute a0509_description in control_node
    robot_description_path = PathJoinSubstitution(
        [
            FindPackageShare("a0509_description"),
            "urdf",
            "a0509.urdf.xacro",
        ]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            robot_description_path,
            " name:=",
            LaunchConfiguration("name"),
            " host:=",
            LaunchConfiguration("host"),
            " rt_host:=",
            LaunchConfiguration("rt_host"),
            " port:=",
            LaunchConfiguration("port"),
            " mode:=",
            LaunchConfiguration("mode"),
            # " update_rate:=",
            # update_rate,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = [
        PathJoinSubstitution(
            [
                FindPackageShare("dsr_controller2"),
                "config",
                "dsr_controller2.yaml",
            ]
        )
    ]

    # CHANGE: hardcode model a0509
    run_emulator_node = Node(
        package="dsr_bringup2",
        executable="run_emulator",
        namespace=LaunchConfiguration("name"),
        parameters=[
            {"name": LaunchConfiguration("name")},
            {"rate": 100},
            {"standby": 5000},
            {"command": True},
            {"host": LaunchConfiguration("host")},
            {"port": LaunchConfiguration("port")},
            {"mode": LaunchConfiguration("mode")},
            {"model": "a0509"},
            {"gripper": "none"},
            {"mobile": "none"},
            {"rt_host": LaunchConfiguration("rt_host")},
            # parameters_file_path       # If a parameter is set in both the launch file and a YAML file, the value from the YAML file will be used.
        ],
        output="screen",
    )

    # CHANGE: add controller_log_level arg to control_node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=LaunchConfiguration("name"),
        parameters=[robot_description, robot_controllers],
        output="both",
        arguments=[
            "--ros-args",
            "--log-level",
            LaunchConfiguration("controller_log_level"),
        ],
    )

    # CHANGE: substitute a0509_description in robot_state_pub_node
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=LaunchConfiguration("name"),
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration("name"),
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration("name"),
        executable="spawner",
        arguments=["dsr_controller2", "-c", "controller_manager"],
    )

    dsr_moveit_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration("name"),
        arguments=[
            "dsr_moveit_controller",
            "-c",
            "controller_manager",
        ],
    )

    # # Moveit2 config
    rviz_node = OpaqueFunction(function=rviz_node_function)

    # CHANGE: zed_wrapper
    zed_wrapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("zed_wrapper"),
                        "launch",
                        "zed_camera.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={"publish_tf": "false", "camera_model": "zed2i"}.items(),
        condition=IfCondition(LaunchConfiguration("launch_zed")),
    )

    # joint_trajectory_controller_spawner = Node(
    #     package="controller_manager",
    #     # namespace=LaunchConfiguration('name'),
    #     executable="spawner",
    #     arguments=["dsr_joint_trajectory", "-c", "dsr/controller_manager", "-n", "dsr"],
    # )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[rviz_node],
        )
    )

    # # Delay start of robot_controller after `joint_state_broadcaster`
    # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[robot_controller_spawner],
    #     )
    # )

    # CHANGE: add zed_wrapper
    nodes = [
        run_emulator_node,
        robot_state_pub_node,
        robot_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        joint_state_broadcaster_spawner,
        dsr_moveit_controller_spawner,
        control_node,
        zed_wrapper,
    ]

    return LaunchDescription(ARGUMENTS + nodes)
