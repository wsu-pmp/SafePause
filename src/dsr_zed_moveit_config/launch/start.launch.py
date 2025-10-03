# Modified from
# https://raw.githubusercontent.com/DoosanRobotics/doosan-robot2/refs/heads/humble/dsr_moveit2/dsr_moveit_config_a0509/launch/start.launch.py


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
from moveit_configs_utils.launches import generate_move_group_launch


def rviz_node_function(context):
    # Moveit2 config replaced with dsr_zed config
    moveit_config = MoveItConfigsBuilder(
        "a0509", package_name="dsr_zed_moveit_config"
    ).to_moveit_configs()
    move_group_launch = generate_move_group_launch(moveit_config)

    # RViz launchreplaced with dsr_zed moveit_rviz launch
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("dsr_zed_moveit_config"),
                        "launch",
                        "moveit_rviz.launch.py",
                    ]
                )
            ]
        ),
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    return [
        *move_group_launch.entities,
        rviz_launch,
    ]


def generate_launch_description():
    ARGUMENTS = [
        DeclareLaunchArgument("name", default_value="", description="NAME_SPACE"),
        DeclareLaunchArgument(
            "host", default_value="127.0.0.1", description="ROBOT_IP"
        ),
        DeclareLaunchArgument("port", default_value="12345", description="ROBOT_PORT"),
        DeclareLaunchArgument(
            "mode", default_value="virtual", description="OPERATION MODE"
        ),
        DeclareLaunchArgument(
            "rt_host", default_value="192.168.137.50", description="ROBOT_RT_IP"
        ),
        DeclareLaunchArgument(
            "launch_rviz", default_value="true", description="Launch RViz?"
        ),
    ]

    def launch_setup(context, *args, **kwargs):
        # Robot description replaced with custom a0509_description
        robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("a0509_description"),
                        "urdf",
                        "a0509.urdf.xacro",
                    ]
                ),
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
            ]
        )

        robot_description = {"robot_description": robot_description_content}

        robot_controllers = PathJoinSubstitution(
            [
                FindPackageShare("dsr_controller2"),
                "config",
                "dsr_controller2.yaml",
            ]
        )

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
            ],
            output="screen",
        )

        control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=LaunchConfiguration("name"),
            parameters=[robot_description, robot_controllers],
            output="both",
        )

        robot_state_pub_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=LaunchConfiguration("name"),
            output="both",
            parameters=[robot_description],  # replaced robot description
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

        # Moveit2 config
        rviz_node = OpaqueFunction(function=rviz_node_function)

        # Delay rviz start after `joint_state_broadcaster`
        delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=robot_controller_spawner,
                on_exit=[rviz_node],
            )
        )

        return [
            run_emulator_node,
            robot_state_pub_node,
            robot_controller_spawner,
            delay_rviz_after_joint_state_broadcaster_spawner,
            joint_state_broadcaster_spawner,
            dsr_moveit_controller_spawner,
            control_node,
        ]

    return LaunchDescription(ARGUMENTS + [OpaqueFunction(function=launch_setup)])
