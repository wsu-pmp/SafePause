import rclpy
from rclpy.node import Node


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")

        self.get_logger().info("perception alive")


def main(args=None):
    rclpy.init(args=args)

    node = PerceptionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupt received. Shutting down.")
    except BaseException as ex:
        node.get_logger().error(ex)
        node.get_logger().debug(ex.with_traceback())
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
