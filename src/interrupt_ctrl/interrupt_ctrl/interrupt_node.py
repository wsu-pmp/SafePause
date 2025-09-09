import traceback

import rclpy
from rclpy.node import Node

NODE_NAME: str = "interrupt_node"


class InterruptNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        self.get_logger().info(f"{NODE_NAME} alive")


def main(args=None):
    rclpy.init(args=args)

    logger = rclpy.logging.get_logger(NODE_NAME)

    node = None
    try:
        node = InterruptNode()
        rclpy.spin(node)
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
