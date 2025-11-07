import threading
import time
import traceback
from dataclasses import dataclass
from queue import Empty, Queue
from typing import Any

import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener
from tf2_sensor_msgs import do_transform_cloud


@dataclass
class PerceptionData:
    timestamp: rclpy.time.Time
    cloud: PointCloud2


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")

        self.declare_parameter("processing_rate", 10.0)  # Hz
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("queue_size", 5)
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop", 0.1)  # seconds
        self.declare_parameter(
            "cloud_topic", "/zed/zed_node/point_cloud/cloud_registered"
        )

        self.processing_rate = self.get_parameter("processing_rate").value
        self.target_frame = self.get_parameter("target_frame").value
        self.queue_size = self.get_parameter("queue_size").value

        # tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # qos profile (shared between sensor topics?)
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.cloud_sub = Subscriber(
            self,
            PointCloud2,
            self.get_parameter("cloud_topic").value,
            qos_profile=sensor_qos,
        )

        # ApproximateTimeSynchronizer for future sensor topics
        self.sync = ApproximateTimeSynchronizer(
            [self.cloud_sub],
            queue_size=self.get_parameter("sync_queue_size").value,
            slop=self.get_parameter("sync_slop").value,
        )
        self.sync.registerCallback(self.sync_callback)

        # processing queue and thread
        self.processing_queue = Queue(maxsize=self.queue_size)
        self.processing_thread = threading.Thread(
            target=self.processing_loop, daemon=True
        )
        self.processing_thread.start()

        self.get_logger().info(
            f"Perception node started:\n"
            f"  Processing rate: {self.processing_rate} Hz\n"
            f"  Target frame: {self.target_frame}\n"
            f"  Queue size: {self.queue_size}\n"
            f"  Sync slop: {self.get_parameter('sync_slop').value}s"
        )

    def sync_callback(self, cloud_msg):
        self.get_logger().warn(
            "cb",
            throttle_duration_sec=2.0,
        )

        try:
            # check if transform is available
            if not self.tf_buffer.can_transform(
                self.target_frame,
                cloud_msg.header.frame_id,
                cloud_msg.header.stamp,
                # rclpy.time.Time(),  # most recent
                timeout=rclpy.duration.Duration(seconds=0.01),
            ):
                self.get_logger().warn(
                    f"Transform not available from {cloud_msg.header.frame_id} "
                    f"to {self.target_frame}",
                    throttle_duration_sec=2.0,
                )
                return

            self.get_logger().warn(
                "Transform available!",
                throttle_duration_sec=2.0,
            )

            # transform point cloud to base_link
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                cloud_msg.header.frame_id,
                cloud_msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.1),
            )

            cloud_transformed = do_transform_cloud(cloud_msg, transform)

            # bundle perception data
            perception_data = PerceptionData(
                timestamp=cloud_msg.header.stamp,
                cloud=cloud_transformed,
            )

            # try to add to processing queue (non-blocking)
            try:
                self.processing_queue.put_nowait(perception_data)
            except Exception as _:
                self.get_logger().warn(
                    "Processing queue full, dropping frame", throttle_duration_sec=1.0
                )

        except Exception as e:
            self.get_logger().warn(
                f"Error in sync_callback: {e}", throttle_duration_sec=1.0
            )

    def processing_loop(self):
        sleep_time = 1.0 / self.processing_rate

        self.get_logger().info("Processing loop started")

        while rclpy.ok():
            try:
                # pull from queue (blocking)
                perception_data = self.processing_queue.get(timeout=1.0)

                start_time = time.time()
                _ = self.process_perception_data(perception_data)

                elapsed = time.time() - start_time

                # warn if processing is slower than target rate
                if elapsed > sleep_time:
                    self.get_logger().warn(
                        f"Processing took {elapsed:.3f}s, target is {sleep_time:.3f}s. "
                        f"Consider reducing processing_rate parameter.",
                        throttle_duration_sec=5.0,
                    )

                # control processing rate
                time.sleep(max(0, sleep_time - elapsed))

            except Empty:
                # no data available, continue waiting
                continue
            except Exception as e:
                self.get_logger().error(f"Processing error: {e}")

    def process_perception_data(self, data: PerceptionData) -> Any:
        self.get_logger().info(
            f"Processing point cloud with {data.cloud.width * data.cloud.height} points",
            throttle_duration_sec=2.0,
        )

        return None


def main(args=None):
    rclpy.init(args=args)

    node = PerceptionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupt received. Shutting down.")
    except BaseException as ex:
        node.get_logger().error(str(ex))
        node.get_logger().debug(traceback.format_exc())
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
