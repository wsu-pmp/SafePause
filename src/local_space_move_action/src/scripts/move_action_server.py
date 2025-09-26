#!/usr/bin/env python3

import argparse

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from local_space_move_action.action import LocalSpaceMove
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, MoveItErrorCodes,
                             OrientationConstraint, PositionConstraint)
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from shape_msgs.msg import SolidPrimitive


class LocalSpaceMoveActionServer(Node):
    MOVE_GROUP_NAME = "manipulator"
    BASE_FRAME_ID = "world"
    END_EFFECTOR_LINK = "link_6"
    PLANNER_ID = "RRTConnect"

    def __init__(self, origin: Pose):
        super().__init__("local_space_move_action_server")

        self.origin: Pose = origin

        # action server (us)
        self._action_server = ActionServer(
            self, LocalSpaceMove, "local_space_move_action", self.execute_callback
        )

        # moveit action client
        self._moveit_client = ActionClient(self, MoveGroup, "/move_action")

        self.get_logger().info("Action server initialized.")
        self.get_logger().info(
            (
                "origin:\n"
                f"\tposition: {self.origin.position}\n"
                f"\torientation: {self.origin.orientation}"
            )
        )
        self.get_logger().info("Awaiting /local_space_move_action action requests.")

    async def execute_callback(self, goal_handle):
        self.get_logger().info(
            (
                "Received goal pose:\n"
                f"\tposition: {goal_handle.request.pose.position}\n"
                f"\torientation: {goal_handle.request.pose.orientation}"
            )
        )

        # transform pose to relative space and create PoseStamped
        transformed_pose: PoseStamped = self.create_pose_stamped(
            goal_handle.request.pose
        )

        self.get_logger().info(
            (
                "Sending MoveIt goal pose:\n"
                f"\tposition: {transformed_pose.pose.position}\n"
                f"\torientation: {transformed_pose.pose.orientation}"
            )
        )

        # create moveit goal
        moveit_goal = self.create_moveit_goal(transformed_pose)

        # wait for moveit action server
        if not self._moveit_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveIt action server not available")
            goal_handle.abort()
            return LocalSpaceMove.Result()

        # send goal
        moveit_future = self._moveit_client.send_goal_async(
            moveit_goal,
            feedback_callback=lambda feedback: self.forward_feedback(
                goal_handle, feedback
            ),
        )

        # await goal result
        try:
            moveit_goal_handle = await moveit_future
        except Exception as e:
            self.get_logger().error(f"MoveIt goal failed: {e}")
            goal_handle.abort()
            return LocalSpaceMove.Result()

        if not moveit_goal_handle.accepted:
            self.get_logger().error("MoveIt goal rejected")
            goal_handle.abort()
            return LocalSpaceMove.Result()

        # wait for result
        result_future = moveit_goal_handle.get_result_async()
        moveit_result = await result_future

        # complete our goal (/local_space_move_action)
        if moveit_result.result.error_code.val == MoveItErrorCodes.SUCCESS:
            goal_handle.succeed()
            self.get_logger().info("MoveIt goal succeeded")
        else:
            goal_handle.abort()
            self.get_logger().warning("MoveIt goal failed")

        # forward moveit result as our result
        return LocalSpaceMove.Result(
            error_code=moveit_result.result.error_code,
            trajectory_start=moveit_result.result.trajectory_start,
            planned_trajectory=moveit_result.result.planned_trajectory,
            executed_trajectory=moveit_result.result.executed_trajectory,
            planning_time=moveit_result.result.planning_time,
        )

    def forward_feedback(self, goal_handle, moveit_feedback):
        # forward moveit feedback as our feedback
        goal_handle.publish_feedback(
            LocalSpaceMove.Feedback(state=moveit_feedback.feedback.state)
        )

    def transform_pose_space(self, local_pose: Pose) -> Pose:
        # build numpy position/orientation vectors from poses
        origin_pos = np.array(
            [
                self.origin.position.x,
                self.origin.position.y,
                self.origin.position.z,
            ]
        )
        origin_orient = [
            self.origin.orientation.x,
            self.origin.orientation.y,
            self.origin.orientation.z,
            self.origin.orientation.w,
        ]
        origin_rot = Rotation.from_quat(origin_orient)

        local_pos = np.array(
            [
                local_pose.position.x,
                local_pose.position.y,
                local_pose.position.z,
            ]
        )
        local_orient = [
            local_pose.orientation.x,
            local_pose.orientation.y,
            local_pose.orientation.z,
            local_pose.orientation.w,
        ]
        local_rot = Rotation.from_quat(local_orient)

        # apply transform
        base_pos = origin_pos + origin_rot.apply(local_pos)
        base_rot = origin_rot * local_rot
        base_orient = base_rot.as_quat()

        # cast numpy.float64 to py floats
        return Pose(
            position=Point(
                x=float(base_pos[0]), y=float(base_pos[1]), z=float(base_pos[2])
            ),
            orientation=Quaternion(
                x=float(base_orient[0]),
                y=float(base_orient[1]),
                z=float(base_orient[2]),
                w=float(base_orient[3]),
            ),
        )

    def create_pose_stamped(self, local_pose: Pose) -> PoseStamped:
        # transform pose to relative space
        relative_pose: Pose = self.transform_pose_space(local_pose)

        transformed_pose = PoseStamped()
        transformed_pose.header.frame_id = self.BASE_FRAME_ID
        transformed_pose.header.stamp = self.get_clock().now().to_msg()
        transformed_pose.pose = relative_pose

        return transformed_pose

    def create_moveit_goal(self, target_pose):
        goal = MoveGroup.Goal()

        # set up the motion plan request
        goal.request.group_name = self.MOVE_GROUP_NAME
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 1.0
        goal.request.max_acceleration_scaling_factor = 1.0

        # position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.BASE_FRAME_ID
        pos_constraint.link_name = self.END_EFFECTOR_LINK
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0

        # constraint region
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.1]  # tolerance used in erica_move_test.move_test_node

        pos_constraint.constraint_region.primitives.append(primitive)
        pos_constraint.constraint_region.primitive_poses.append(target_pose.pose)
        pos_constraint.weight = 1.0

        # orientation constraint
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = self.BASE_FRAME_ID
        orient_constraint.link_name = self.END_EFFECTOR_LINK
        orient_constraint.orientation = target_pose.pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.01
        orient_constraint.absolute_y_axis_tolerance = 0.01
        orient_constraint.absolute_z_axis_tolerance = 0.01
        orient_constraint.weight = 1.0

        # apply constraints
        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(orient_constraint)
        goal.request.goal_constraints.append(constraints)

        # general planning options
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.look_around_attempts = 0
        goal.planning_options.max_safe_execution_cost = 0.0
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 5

        return goal


def main():
    origin_arg_defaults = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

    parser = argparse.ArgumentParser(description="Local Space Move Action Server")
    parser.add_argument(
        "--origin",
        type=float,
        nargs="*",
        default=origin_arg_defaults,
        help="Local space origin pose: x y z qx qy qz qw (defaults: 0.0 0.0 0.0 0.0 0.0 0.0 1.0)",
    )

    args, unknown = parser.parse_known_args()

    # extend omitted args with defaults
    if len(args.origin) < 7:
        args.origin.extend(origin_arg_defaults[len(args.origin):])

    origin_pose: Pose = Pose(
        position=Point(x=args.origin[0], y=args.origin[1], z=args.origin[2]),
        orientation=Quaternion(
            x=args.origin[3], y=args.origin[4], z=args.origin[5], w=args.origin[6]
        ),
    )

    rclpy.init(args=unknown)

    node = LocalSpaceMoveActionServer(origin_pose)

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
