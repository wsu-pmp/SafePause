import traceback
from abc import ABC, abstractmethod
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA

NODE_NAME: str = "planning_scene_loader"


class ShapeDefinition(ABC):
    def __init__(
        self,
        name: str,
        frame_id: str,
        position: list,
        orientation: list,
        attached_link: str = None,
    ):
        self.name = name
        self.frame_id = frame_id
        self.position = position
        self.orientation = orientation  # quaternion [x, y, z, w]
        self.attached_link = attached_link

    def create_pose(self) -> Pose:
        pose = Pose()
        pose.position = Point(
            x=self.position[0], y=self.position[1], z=self.position[2]
        )
        pose.orientation = Quaternion(
            x=self.orientation[0],
            y=self.orientation[1],
            z=self.orientation[2],
            w=self.orientation[3],
        )
        return pose

    @abstractmethod
    def create_collision_object(self) -> CollisionObject:
        pass


class BoxShape(ShapeDefinition):
    def __init__(
        self,
        name: str,
        frame_id: str,
        position: list,
        orientation: list,
        dimensions: list,
        attached_link: str = None,
    ):
        super().__init__(name, frame_id, position, orientation, attached_link)
        self.dimensions = dimensions  # [x, y, z]

    def create_collision_object(self) -> CollisionObject:
        collision_obj = CollisionObject()
        collision_obj.header.frame_id = self.frame_id
        collision_obj.id = self.name

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = self.dimensions

        collision_obj.primitives.append(primitive)
        collision_obj.primitive_poses.append(self.create_pose())
        collision_obj.operation = CollisionObject.ADD

        return collision_obj


class SphereShape(ShapeDefinition):
    def __init__(
        self,
        name: str,
        frame_id: str,
        position: list,
        orientation: list,
        radius: float,
        attached_link: str = None,
    ):
        super().__init__(name, frame_id, position, orientation, attached_link)
        self.radius = radius

    def create_collision_object(self) -> CollisionObject:
        collision_obj = CollisionObject()
        collision_obj.header.frame_id = self.frame_id
        collision_obj.id = self.name

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [self.radius]

        collision_obj.primitives.append(primitive)
        collision_obj.primitive_poses.append(self.create_pose())
        collision_obj.operation = CollisionObject.ADD

        return collision_obj


class CylinderShape(ShapeDefinition):
    def __init__(
        self,
        name: str,
        frame_id: str,
        position: list,
        orientation: list,
        height: float,
        radius: float,
        attached_link: str = None,
    ):
        super().__init__(name, frame_id, position, orientation, attached_link)
        self.height = height
        self.radius = radius

    def create_collision_object(self) -> CollisionObject:
        collision_obj = CollisionObject()
        collision_obj.header.frame_id = self.frame_id
        collision_obj.id = self.name

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [self.height, self.radius]

        collision_obj.primitives.append(primitive)
        collision_obj.primitive_poses.append(self.create_pose())
        collision_obj.operation = CollisionObject.ADD

        return collision_obj


class PlaneShape(ShapeDefinition):
    def __init__(
        self,
        name: str,
        frame_id: str,
        position: list,
        orientation: list,
        width: float,
        length: float,
        thickness: float = 0.01,
        attached_link: str = None,
    ):
        super().__init__(name, frame_id, position, orientation, attached_link)
        self.width = width
        self.length = length
        self.thickness = thickness

    def create_collision_object(self) -> CollisionObject:
        collision_obj = CollisionObject()
        collision_obj.header.frame_id = self.frame_id
        collision_obj.id = self.name

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [self.width, self.length, self.thickness]

        collision_obj.primitives.append(primitive)
        collision_obj.primitive_poses.append(self.create_pose())
        collision_obj.operation = CollisionObject.ADD

        return collision_obj


class ShapeFactory:
    @staticmethod
    def create_shape(shape_config: dict) -> ShapeDefinition:
        shape_type = shape_config["type"].lower()
        name = shape_config["name"]
        frame_id = shape_config.get("frame_id", "world")
        position = shape_config.get("position", [0.0, 0.0, 0.0])
        orientation = shape_config.get("orientation", [0.0, 0.0, 0.0, 1.0])
        attached_link = shape_config.get("attached_link", None)

        if shape_type == "box":
            dimensions = shape_config["dimensions"]  # [x, y, z]
            return BoxShape(
                name, frame_id, position, orientation, dimensions, attached_link
            )

        elif shape_type == "sphere":
            radius = shape_config["radius"]
            return SphereShape(
                name, frame_id, position, orientation, radius, attached_link
            )

        elif shape_type == "cylinder":
            height = shape_config["height"]
            radius = shape_config["radius"]
            return CylinderShape(
                name, frame_id, position, orientation, height, radius, attached_link
            )

        elif shape_type == "plane":
            width = shape_config["width"]
            length = shape_config["length"]
            thickness = shape_config.get("thickness", 0.01)
            return PlaneShape(
                name,
                frame_id,
                position,
                orientation,
                width,
                length,
                thickness,
                attached_link,
            )

        else:
            raise ValueError(f"Unknown shape type: {shape_type}")


class PlanningSceneLoader(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        self.declare_parameter("config_file", "")

        config_file = (
            self.get_parameter("config_file").get_parameter_value().string_value
        )

        if not config_file:
            self.get_logger().error(
                "No config file specified. Use --ros-args -p config_file:=<path>"
            )
            raise RuntimeError("No config file specified")

        # load config
        self.shapes = self.load_config(config_file)

        if not self.shapes:
            self.get_logger().warn("No shapes loaded from configuration")
            return  # Nothing to do, just exit gracefully

        # service client for applying planning scene
        self.planning_scene_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )

        self.get_logger().info("Waiting for /apply_planning_scene service...")
        if not self.planning_scene_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Service /apply_planning_scene not available")
            raise RuntimeError("Planning scene service not available")

        self.get_logger().info("Service available, adding shapes to planning scene...")
        self.add_shapes_to_scene()

    def load_config(self, config_file: str) -> list:
        try:
            config_path = Path(config_file)
            if not config_path.exists():
                self.get_logger().error(f"Config file not found: {config_file}")
                raise FileNotFoundError(f"Config file not found: {config_file}")

            with open(config_path, "r") as f:
                config = yaml.safe_load(f)

            shapes = []
            if "shapes" in config:
                for shape_config in config["shapes"]:
                    try:
                        shape = ShapeFactory.create_shape(shape_config)
                        shapes.append(shape)
                        self.get_logger().info(
                            f"Loaded shape: {shape.name} (type: {shape_config['type']})"
                        )
                    except Exception as e:
                        self.get_logger().error(f"Failed to create shape: {e}")

            return shapes

        except Exception as e:
            self.get_logger().error(f"Failed to load config file: {e}")
            raise

    def add_shapes_to_scene(self):
        planning_scene_msg = PlanningScene()
        planning_scene_msg.is_diff = True

        # separate attached and world objects
        attached_objects = []
        world_objects = []

        # create collision objects for all shapes
        for shape in self.shapes:
            collision_obj: CollisionObject = shape.create_collision_object()

            # handle attached objects
            if shape.attached_link:
                from moveit_msgs.msg import AttachedCollisionObject

                attached_obj = AttachedCollisionObject()
                attached_obj.link_name = shape.attached_link
                attached_obj.object = collision_obj
                attached_objects.append(attached_obj)
                self.get_logger().info(
                    f"Adding attached shape: {shape.name} to link {shape.attached_link}"
                )
            else:
                world_objects.append(collision_obj)
                self.get_logger().info(f"Adding shape: {shape.name}")

            obj_color = ObjectColor()
            obj_color.id = collision_obj.id

            obj_color.color = ColorRGBA()
            obj_color.color.r = 0.8
            obj_color.color.g = 0.0
            obj_color.color.b = 0.75
            obj_color.color.a = 0.8
            planning_scene_msg.object_colors.append(obj_color)

        # only populate robot_state if we have attached objects
        if attached_objects:
            # Create a proper robot_state with a valid (even if empty) joint_state
            planning_scene_msg.robot_state.joint_state.header.stamp = (
                self.get_clock().now().to_msg()
            )
            planning_scene_msg.robot_state.attached_collision_objects = attached_objects
            planning_scene_msg.robot_state.is_diff = True

        # add world objects
        planning_scene_msg.world.collision_objects = world_objects

        request = ApplyPlanningScene.Request()
        request.scene = planning_scene_msg

        future = self.planning_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(
                    f"Successfully added {len(self.shapes)} shapes to planning scene"
                )
            else:
                self.get_logger().error("Failed to apply planning scene")
                raise RuntimeError("Failed to apply planning scene")
        else:
            self.get_logger().error("Service call failed")
            raise RuntimeError("Service call to apply_planning_scene failed")


def main(args=None):
    rclpy.init(args=args)

    logger = rclpy.logging.get_logger(NODE_NAME)

    node = None
    try:
        node = PlanningSceneLoader()
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
