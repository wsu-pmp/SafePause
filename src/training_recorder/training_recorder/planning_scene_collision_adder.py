#!/usr/bin/env python3

import os
from typing import Any, Dict
from uuid import uuid4

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import Plane, SolidPrimitive


class PlanningSceneCollisionAdder(Node):
    def __init__(self):
        super().__init__("planning_scene_collision_adder")

        pkg_share = get_package_share_directory("training_recorder")
        param_file = os.path.join(pkg_share, "config", "collision_objects.yaml")

        self.declare_parameter("collision_objects_config", param_file)

        thing = (
            self.get_parameter("collision_objects_config")
            .get_parameter_value()
            .string_value
        )

        if not os.path.isfile(thing):
            self.get_logger().error(f"Invalid collision_objects_config: '{thing}'")
            return

        with open(thing, "r") as f:
            try:
                d: Dict[str, Any] = yaml.safe_load(f)
            except Exception:
                self.get_logger().error(f"Unable to load yaml: '{thing}'")
                return

        if (collision_objects := d.get("collision_objects")) is None:
            self.get_logger().error(f"'collision_objects' missing from yaml: '{thing}'")
            return

        self.collision_objects = collision_objects

        # Wait for MoveIt to be ready
        self.scene_service = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )

        # Wait for service to be available
        while not self.scene_service.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for /apply_planning_scene service...")

        # Add collision objects
        self.add_collision_objects()

    def add_collision_objects(self):
        # Create planning scene request
        planning_scene = PlanningScene()
        planning_scene.is_diff = True

        primitives = self.collision_objects.get("primitives")
        if primitives is not None:
            for item in primitives:
                primitive = item.get("primitive")
                pose = item.get("pose")

                collision_object = CollisionObject()
                collision_object.operation = CollisionObject.ADD
                collision_object.header.frame_id = item.get("frame_id")
                collision_object.id = f"primitive_{uuid4()}"

                try:
                    obj_primitve = SolidPrimitive()
                    obj_primitve.type = getattr(
                        SolidPrimitive, primitive.get("type").upper()
                    )
                    obj_primitve.dimensions = [
                        float(v) for v in primitive.get("dimensions")
                    ]

                    obj_pose = Pose()
                    pos = pose.get("position")
                    orient = pose.get("orientation")
                    obj_pose.position = Point(
                        x=float(pos.get("x")),
                        y=float(pos.get("y")),
                        z=float(pos.get("z")),
                    )
                    obj_pose.orientation = Quaternion(
                        x=float(orient.get("x")),
                        y=float(orient.get("y")),
                        z=float(orient.get("z")),
                        w=float(orient.get("w")),
                    )

                    # add to collision_object
                    collision_object.primitives.append(obj_primitve)
                    collision_object.primitive_poses.append(obj_pose)

                    # add collision_object to planning_scene
                    planning_scene.world.collision_objects.append(collision_object)

                except Exception as ex:
                    print(f"ree: {ex}")
                    continue

        planes = self.collision_objects.get("planes")
        if planes is not None:
            for item in planes:
                plane = item.get("plane")
                pose = item.get("pose")

                collision_object = CollisionObject()
                collision_object.operation = CollisionObject.ADD
                collision_object.header.frame_id = item.get("frame_id")
                collision_object.id = f"plane_{uuid4()}"

                try:
                    obj_plane = Plane()
                    obj_plane.coef = [float(v) for v in plane.get("coef")]

                    obj_pose = Pose()
                    pos = pose.get("position")
                    orient = pose.get("orientation")
                    obj_pose.position = Point(
                        x=float(pos.get("x")),
                        y=float(pos.get("y")),
                        z=float(pos.get("z")),
                    )
                    obj_pose.orientation = Quaternion(
                        x=float(orient.get("x")),
                        y=float(orient.get("y")),
                        z=float(orient.get("z")),
                        w=float(orient.get("w")),
                    )

                    # add to collision_object
                    collision_object.planes.append(obj_plane)
                    collision_object.plane_poses.append(obj_pose)

                    # add collision_object to planning_scene
                    planning_scene.world.collision_objects.append(collision_object)

                except Exception as ex:
                    print(f"ree: {ex}")
                    continue

        # Send request
        request = ApplyPlanningScene.Request()
        request.scene = planning_scene

        future = self.scene_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info("Collsion objects added to planning scene")
        else:
            self.get_logger().error("Failed to add collsion objects to planning scene")


def main():
    rclpy.init()

    node = PlanningSceneCollisionAdder()

    # Keep node alive briefly, then shutdown
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
