import random
import traceback
from typing import Optional

import numpy as np
import rclpy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import (
    Point,
    PoseStamped,
)
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from tf2_ros import TransformException
from zed_msgs.msg import ObjectsStamped

from event_logger import EventLogger

NODE_NAME: str = "object_approach_node"


class ObjectApproachNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        self.declare_parameter("max_range", 1.05)  # meters
        self.declare_parameter("approach_distance", 0.05)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("planning_group", "manipulator")
        self.declare_parameter("end_effector_link", "link_6")
        self.declare_parameter("objects_topic", "/obj_det/small_objects")
        self.declare_parameter("move_action", "/move_action")
        self.declare_parameter("min_accumulation", 3)
        self.declare_parameter("move_action_timeout", 5.0)  # seconds
        self.declare_parameter("tf_timeout", 1.0)  # seconds

        self.max_range = self.get_parameter("max_range").value
        self.approach_distance = self.get_parameter("approach_distance").value
        self.base_frame = self.get_parameter("base_frame").value
        self.planning_group = self.get_parameter("planning_group").value
        self.end_effector_link = self.get_parameter("end_effector_link").value
        self.objects_topic = self.get_parameter("objects_topic").value
        self.move_action = self.get_parameter("move_action").value
        self.min_accumulation = self.get_parameter("min_accumulation").value
        self.move_action_timeout = self.get_parameter("move_action_timeout").value
        self.tf_timeout = self.get_parameter("tf_timeout").value

        self.events = EventLogger(
            self,
            NODE_NAME,
            {
                "max_range": self.max_range,
                "approach_distance": self.approach_distance,
                "base_frame": self.base_frame,
                "planning_group": self.planning_group,
                "end_effector_link": self.end_effector_link,
                "objects_topic": self.objects_topic,
                "move_action": self.move_action,
                "min_accumulation": self.min_accumulation,
                "move_action_timeout": self.move_action_timeout,
                "tf_timeout": self.tf_timeout,
            },
        )

        # tf2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # action client
        self._move_action_client = ActionClient(self, MoveGroup, self.move_action)

        # state
        self.goal_sent = False
        self.accumulated_objects = []
        self.accumulation_count = 0

        # exclusive group - one detection cycle at a time, but in parallel with
        # the listener's tf callbacks
        self.subscription = self.create_subscription(
            ObjectsStamped,
            self.objects_topic,
            self.objects_callback,
            10,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.get_logger().info("Object Approach Node initialized")
        self.get_logger().info(
            f"Max range: {self.max_range}m, Approach distance: {self.approach_distance}m"
        )
        self.get_logger().info(f"Event log: {self.events.path}")

    def destroy_node(self):
        self.events.close()
        super().destroy_node()

    def objects_callback(self, msg: ObjectsStamped):
        if self.goal_sent:
            return  # one shot operation, already sent goal

        # accumulate objects
        self.accumulated_objects.append(msg)
        self.accumulation_count += 1

        if self.accumulation_count < self.min_accumulation:
            self.get_logger().info(
                f"Accumulating objects ({self.accumulation_count}/{self.min_accumulation})..."
            )
            return

        # select object and send goal
        self.process_accumulated_objects()

    def process_accumulated_objects(self):
        # flatten objects from accumulated messages
        all_objects = []
        for msg in self.accumulated_objects:
            for obj in msg.objects:
                all_objects.append((obj, msg.header))

        if not all_objects:
            self.get_logger().warn("No objects detected in accumulated messages")
            self.events.log(
                {"event": "cycle_empty", "messages": len(self.accumulated_objects)}
            )
            return

        self.get_logger().info(f"Processing {len(all_objects)} total detected objects")

        # filter within range. counts are aggregated into one cycle event rather
        # than logged per object, so the log stays proportionate to cycles
        objects_in_range = []
        lookup_failures = 0
        for obj, header in all_objects:
            try:
                # transform object position to base_link
                distance = self.get_object_distance(obj, header)
                if distance is None:
                    lookup_failures += 1
                elif distance <= self.max_range:
                    objects_in_range.append((obj, header, distance))
            except Exception as e:
                lookup_failures += 1
                self.get_logger().warn(f"Failed to process object: {e}")
                continue

        self.events.log(
            {
                "event": "cycle_processed",
                "messages": len(self.accumulated_objects),
                "objects": len(all_objects),
                "in_range": len(objects_in_range),
                "lookup_failures": lookup_failures,
            }
        )

        if not objects_in_range:
            self.get_logger().warn(
                f"No objects within range ({self.max_range}m) of {self.base_frame}"
            )
            return

        self.get_logger().info(f"Found {len(objects_in_range)} objects within range")

        # random choice
        selected_obj, selected_header, distance = random.choice(objects_in_range)
        self.get_logger().info(
            f"Selected object: {selected_obj.label} at {distance:.2f}m"
        )
        self.events.log(
            {
                "event": "object_selected",
                "label": selected_obj.label,
                "distance": float(distance),
                "frame_id": selected_header.frame_id,
            }
        )

        approach_pose = self.compute_approach_pose(selected_obj, selected_header)

        if approach_pose is None:
            self.get_logger().error("Failed to compute approach pose")
            self.events.log({"event": "approach_pose_failed"})
            return

        self.goal_sent = self.send_moveit_goal(approach_pose)

    def get_object_distance(self, obj, header: Header) -> Optional[float]:
        try:
            # get transform from object frame to base_link
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                header.frame_id,
                Time.from_msg(header.stamp),
                timeout=Duration(seconds=self.tf_timeout),
            )

            # pos in objects own frame
            obj_point = Point()
            obj_point.x = float(obj.position[0])
            obj_point.y = float(obj.position[1])
            obj_point.z = float(obj.position[2])

            # transform
            obj_point_stamped = tf2_geometry_msgs.PoseStamped()
            obj_point_stamped.header = header
            obj_point_stamped.pose.position = obj_point
            obj_point_stamped.pose.orientation.w = 1.0

            transformed = tf2_geometry_msgs.do_transform_pose(
                obj_point_stamped.pose, transform
            )

            # calc distance
            distance = np.sqrt(
                transformed.position.y**2
                + transformed.position.x**2
                + transformed.position.z**2
            )

            return distance

        except TransformException as e:
            self.get_logger().warn(f"Transform failed: {e}")
            return None

    def compute_approach_pose(self, obj, header: Header) -> Optional[PoseStamped]:
        try:
            # get transform to base_link
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                header.frame_id,
                Time.from_msg(header.stamp),
                timeout=Duration(seconds=self.tf_timeout),
            )

            # transform corners of bounding box to base_link
            bbox = obj.bounding_box_3d
            corners_base = []

            for corner in bbox.corners:
                corner_point = Point()
                corner_point.x = float(corner.kp[0])
                corner_point.y = float(corner.kp[1])
                corner_point.z = float(corner.kp[2])

                corner_stamped = tf2_geometry_msgs.PoseStamped()
                corner_stamped.header = header
                corner_stamped.pose.position = corner_point
                corner_stamped.pose.orientation.w = 1.0

                transformed = tf2_geometry_msgs.do_transform_pose(
                    corner_stamped.pose, transform
                )

                corners_base.append(
                    [
                        transformed.position.x,
                        transformed.position.y,
                        transformed.position.z,
                    ]
                )

            corners_base = np.array(corners_base)

            # define vertical faces by their vertices using zed_msgs/BoundingBox3D
            # corner indices
            vertical_faces = [
                [0, 1, 5, 4],  # face 1
                [1, 2, 6, 5],  # face 2
                [2, 3, 7, 6],  # face 3
                [3, 0, 4, 7],  # face 4
            ]

            best_face = None
            best_score = -np.inf

            for face_indices in vertical_faces:
                face_corners = corners_base[face_indices]

                # compute face center
                face_center = np.mean(face_corners, axis=0)

                # compute face normal (cross product of two edges)
                edge1 = face_corners[1] - face_corners[0]
                edge2 = face_corners[3] - face_corners[0]
                normal = np.cross(edge1, edge2)
                normal = normal / np.linalg.norm(normal)

                # ensure normal points outward (away from bbox center)
                bbox_center = np.mean(corners_base, axis=0)
                if np.dot(normal, face_center - bbox_center) < 0:
                    normal = -normal

                # score: prefer faces pointing towards base_link (origin)
                # direction from face center to origin
                to_origin = -face_center
                to_origin = to_origin / np.linalg.norm(to_origin)

                # dot product: how much face normal aligns with direction to origin
                score = np.dot(normal, to_origin)

                if score > best_score:
                    best_score = score
                    best_face = {
                        "center": face_center,
                        "normal": normal,
                        "score": score,
                    }

            if best_face is None:
                self.get_logger().error("Could not determine best face")
                return None

            self.get_logger().info(
                f"Selected face with score {best_face['score']:.2f} "
                f"(1.0 = directly facing robot)"
            )

            # compute approach pose
            # position: face center + approach_distance along outward normal
            approach_position = (
                best_face["center"] + best_face["normal"] * self.approach_distance
            )

            # orientation: z axis of ee should align with face normal
            # (pointing into the face), keeping ee upright relative to base_link
            z_axis = -best_face["normal"]  # towards face
            z_axis = z_axis / np.linalg.norm(z_axis)

            # keep y axis aligned with base_link's vertical
            world_up = np.array([0, 0, -1])

            # project world_up onto plane perpendicular to z_axis
            y_axis = world_up - np.dot(world_up, z_axis) * z_axis

            if np.linalg.norm(y_axis) < 0.01:  # z nearly vertical
                # use projection of x-direction onto plane perpendicular to z
                world_x = np.array([1, 0, 0])
                y_axis = world_x - np.dot(world_x, z_axis) * z_axis
                if np.linalg.norm(y_axis) < 0.01:
                    y_axis = np.array([0, 1, 0])

            y_axis = y_axis / np.linalg.norm(y_axis)

            # x axis given by other two
            x_axis = np.cross(y_axis, z_axis)
            x_axis = x_axis / np.linalg.norm(x_axis)

            # build rotation matrix
            rotation_matrix = np.column_stack([x_axis, y_axis, z_axis])

            # convert to quaternion
            quat = self.rotation_matrix_to_quaternion(rotation_matrix)

            # PoseStamped
            approach_pose = PoseStamped()
            approach_pose.header.frame_id = self.base_frame
            approach_pose.header.stamp = self.get_clock().now().to_msg()

            approach_pose.pose.position.x = approach_position[0]
            approach_pose.pose.position.y = approach_position[1]
            approach_pose.pose.position.z = approach_position[2]

            approach_pose.pose.orientation.x = quat[0]
            approach_pose.pose.orientation.y = quat[1]
            approach_pose.pose.orientation.z = quat[2]
            approach_pose.pose.orientation.w = quat[3]

            self.get_logger().info(
                f"Approach pose: pos=[{approach_position[0]:.3f}, "
                f"{approach_position[1]:.3f}, {approach_position[2]:.3f}]"
            )

            return approach_pose

        except Exception as e:
            self.get_logger().error(f"Failed to compute approach pose: {e}")
            self.get_logger().error(traceback.format_exc())
            return None

    @staticmethod
    def rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
        trace = np.trace(R)

        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s

        return np.array([x, y, z, w])

    def send_moveit_goal(self, target_pose: PoseStamped) -> bool:
        self.get_logger().info("Waiting for MoveGroup action server...")

        if not self._move_action_client.wait_for_server(
            timeout_sec=self.move_action_timeout
        ):
            self.get_logger().error(
                f"MoveGroup action server '{self.move_action}' unavailable after "
                f"{self.move_action_timeout}s; will retry on further detections"
            )
            self.events.log(
                {
                    "event": "move_action_unavailable",
                    "move_action": self.move_action,
                    "timeout": self.move_action_timeout,
                }
            )
            return False

        goal_msg = MoveGroup.Goal()

        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = self.planning_group
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1

        goal_constraint = Constraints()
        goal_constraint.name = "approach_pose"

        # position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header = target_pose.header
        pos_constraint.link_name = self.end_effector_link

        # position tolerance via small box around target
        pos_constraint.constraint_region = BoundingVolume()
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.01, 0.01, 0.01]  # 1cm tolerance
        pos_constraint.constraint_region.primitives = [box]
        pos_constraint.constraint_region.primitive_poses = [target_pose.pose]
        pos_constraint.weight = 1.0

        goal_constraint.position_constraints = [pos_constraint]

        # orientation constraint
        orient_constraint = OrientationConstraint()
        orient_constraint.header = target_pose.header
        orient_constraint.link_name = self.end_effector_link
        orient_constraint.orientation = target_pose.pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.1  # radians
        orient_constraint.absolute_y_axis_tolerance = 0.1
        orient_constraint.absolute_z_axis_tolerance = 0.1
        orient_constraint.weight = 1.0

        goal_constraint.orientation_constraints = [orient_constraint]

        goal_msg.request.goal_constraints = [goal_constraint]

        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 5
        goal_msg.planning_options.replan_delay = 2.0

        self.get_logger().info("Sending MoveGroup goal...")
        self.events.log(
            {
                "event": "goal_sent",
                "position": [
                    target_pose.pose.position.x,
                    target_pose.pose.position.y,
                    target_pose.pose.position.z,
                ],
                "frame_id": target_pose.header.frame_id,
                "planning_group": self.planning_group,
            }
        )

        send_goal_future = self._move_action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by action server")
            self.events.log({"event": "goal_rejected"})
            return

        self.get_logger().info("Goal accepted by action server")
        self.events.log({"event": "goal_accepted"})

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Feedback: state={feedback.state}", throttle_duration_sec=1.0
        )

    def result_callback(self, future):
        result = future.result().result

        succeeded = result.error_code.val == result.error_code.SUCCESS
        if succeeded:
            self.get_logger().info("Motion planning and execution succeeded!")
        else:
            self.get_logger().error(
                f"Motion planning failed with error code: {result.error_code.val}"
            )

        self.events.log(
            {
                "event": "result",
                "succeeded": succeeded,
                "error_code": result.error_code.val,
            }
        )

        # shutdown after single run
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    logger = rclpy.logging.get_logger(NODE_NAME)

    node = None
    try:
        node = ObjectApproachNode()
        # multi threaded so tf callbacks can run while a detection cycle blocks
        # in a transform lookup
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        logger.info("Interrupt received. Shutting down.")
    except BaseException as ex:
        logger.error(str(ex))
        logger.debug(traceback.format_exc())
    finally:
        if node:
            node.destroy_node()

        if rclpy.ok():
            rclpy.try_shutdown()
