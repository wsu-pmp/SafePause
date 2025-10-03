import argparse
import threading
import time
import traceback

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.serialization import serialize_message
from rosbag2_py import ConverterOptions, SequentialWriter, StorageOptions, TopicMetadata
from rosidl_runtime_py.utilities import get_message
from std_srvs.srv import Trigger

NODE_NAME: str = "rosbag2_recorder"


class Rosbag2RecorderNode(Node):
    def __init__(self, namespace: str):
        ns_append = f"_{namespace}" if namespace else ""
        super().__init__(f"{NODE_NAME}{ns_append}")

        self.declare_parameter("topics", [""])
        self.declare_parameter("output_dir", "")
        self.declare_parameter("qos_overrides_yaml", "")
        self.declare_parameter("require_all_topics", True)

        self.topics = self.get_parameter("topics").value
        self.output_dir = self.get_parameter("output_dir").value
        self.require_all_topics = self.get_parameter("require_all_topics").value

        qos_overrides_yaml = self.get_parameter("qos_overrides_yaml").value
        self.qos_overrides = (
            yaml.safe_load(qos_overrides_yaml) if qos_overrides_yaml else {}
        )

        if len(self.topics) == 0 or self.topics[0] == "":
            raise ValueError("topics parameter is required")
        if not self.output_dir:
            raise ValueError("output_dir parameter is required")

        # state
        self.recording = False
        self.recording_lock = threading.Lock()
        self.writer = None
        self.subscribers = {}
        self.topic_types = {}
        self.shutdown_timer = None

        self._setup_writer()

        # block until subscriptions are discovered or timeout
        self._setup_subscriptions_with_timeout()

        # create services after subscription discovery completes
        self.start_srv = self.create_service(Trigger, "~/start", self.start_callback)
        self.pause_srv = self.create_service(Trigger, "~/pause", self.pause_callback)
        self.stop_srv = self.create_service(Trigger, "~/stop", self.stop_callback)

        self.get_logger().info(f"Recorder initialized for topics: {self.topics}")

    def _setup_writer(self):
        storage_options = StorageOptions(uri=self.output_dir, storage_id="mcap")

        converter_options = ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        )

        self.writer = SequentialWriter()
        self.writer.open(storage_options, converter_options)

    def _setup_subscriptions_with_timeout(self):
        timeout = 5.0  # seconds
        start_time = time.time()

        undiscovered_topics = set(self.topics)

        while undiscovered_topics and (time.time() - start_time) < timeout:
            topics_to_remove = []

            for topic in undiscovered_topics:
                if self._try_subscribe(topic):
                    topics_to_remove.append(topic)

            for topic in topics_to_remove:
                undiscovered_topics.remove(topic)

            if undiscovered_topics:
                time.sleep(0.1)

        if undiscovered_topics:
            missing = sorted(undiscovered_topics)
            if self.require_all_topics:
                raise RuntimeError(
                    f"Failed to discover topics after {timeout}s: {missing}"
                )

            self.get_logger().error(
                f"Failed to discover topics after {timeout}s, "
                f"nothing will be recorded for them: {missing}"
            )

    @staticmethod
    def _enum_from_name(enum_cls, field, value):
        try:
            return enum_cls[value.upper()]
        except KeyError:
            valid = ", ".join(m.lower() for m in enum_cls.__members__ if m != "UNKNOWN")
            raise ValueError(
                f"invalid qos {field} '{value}'; expected one of: {valid}"
            ) from None

    def _qos_from_override(self, override):
        profile = QoSProfile(depth=override.get("depth", 10))

        if value := override.get("reliability"):
            profile.reliability = self._enum_from_name(
                QoSReliabilityPolicy, "reliability", value
            )
        if value := override.get("durability"):
            profile.durability = self._enum_from_name(
                QoSDurabilityPolicy, "durability", value
            )
        if value := override.get("history"):
            profile.history = self._enum_from_name(QoSHistoryPolicy, "history", value)

        return profile

    def _resolve_qos(self, topic):
        if override := self.qos_overrides.get(topic):
            return self._qos_from_override(override)

        profile = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # choose the most permissive QoS setting a publisher offers
        infos = self.get_publishers_info_by_topic(topic)
        if not infos:
            profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
            profile.durability = QoSDurabilityPolicy.VOLATILE
            return profile

        for info in infos:
            if info.qos_profile.reliability == QoSReliabilityPolicy.BEST_EFFORT:
                profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
            if info.qos_profile.durability == QoSDurabilityPolicy.VOLATILE:
                profile.durability = QoSDurabilityPolicy.VOLATILE

        return profile

    def _try_subscribe(self, topic):
        try:
            topic_types = self.get_topic_names_and_types()

            topic_type = None
            for t, types in topic_types:
                if t == topic:
                    if types:
                        topic_type = types[0]
                        break

            if not topic_type:
                return False

            # get message class
            msg_class = get_message(topic_type)
            qos_profile = self._resolve_qos(topic)

            sub = self.create_subscription(
                msg_class,
                topic,
                lambda msg, t=topic: self._message_callback(msg, t),
                qos_profile,
            )

            self.writer.create_topic(
                TopicMetadata(name=topic, type=topic_type, serialization_format="cdr")
            )

            self.topic_types[topic] = topic_type
            self.subscribers[topic] = sub

            self.get_logger().info(
                f"Subscribed to {topic} ({topic_type}) as "
                f"{qos_profile.reliability.name}/{qos_profile.durability.name}"
            )
            return True

        except Exception as e:
            self.get_logger().error(f"Failed to subscribe to {topic}: {e}")
            self.get_logger().debug(traceback.format_exc())
            return False

    def _message_callback(self, msg, topic):
        if not self.recording:
            return

        try:
            # serialize outside the lock
            serialized_msg = serialize_message(msg)

            # receipt time
            timestamp = self.get_clock().now().nanoseconds

            with self.recording_lock:
                if not self.recording or self.writer is None:
                    return
                self.writer.write(topic, serialized_msg, timestamp)

            self.get_logger().debug(f"Wrote message from {topic} at {timestamp}")
        except Exception as e:
            self.get_logger().error(
                f"Error writing message from {topic}: {e}",
                throttle_duration_sec=1.0,
            )

    def _shutdown(self):
        if self.shutdown_timer:
            self.shutdown_timer.cancel()

        threading.Thread(target=rclpy.shutdown, daemon=True).start()

    def start_callback(self, request, response):
        with self.recording_lock:
            if self.recording:
                response.success = False
                response.message = "Already recording"
            else:
                self.recording = True
                response.success = True
                response.message = "Recording started"
                self.get_logger().info("Recording started")

        return response

    def pause_callback(self, request, response):
        with self.recording_lock:
            if not self.recording:
                response.success = False
                response.message = "Not currently recording"
            else:
                self.recording = False
                response.success = True
                response.message = "Recording paused"
                self.get_logger().info("Recording paused")

        return response

    def stop_callback(self, request, response):
        with self.recording_lock:
            if not self.recording:
                response.success = False
                response.message = "Recording has not been started"

                return response

            self.recording = False

            try:
                if self.writer:
                    self.writer.close()
                    # explicitly delete writer to ensure all resources are released
                    del self.writer
                    self.writer = None
                    self.get_logger().info("Recording stopped, bag closed")

                # schedule shutdown after this callback returns
                self.shutdown_timer = self.create_timer(
                    0.0,  # next spin
                    self._shutdown,
                )

                response.success = True
                response.message = "Recording stopped successfully"
            except Exception as e:
                self.get_logger().error(f"Error stopping recording: {e}")
                response.success = False
                response.message = f"Error stopping: {str(e)}"

        return response

    def destroy_node(self):
        with self.recording_lock:
            if self.writer:
                try:
                    self.writer.close()
                    self.get_logger().info("Writer closed on shutdown")
                except BaseException as _:
                    pass

        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--namespace", required=False)
    args, unknown = parser.parse_known_args()

    rclpy.init(args=unknown)

    logger = rclpy.logging.get_logger(NODE_NAME)

    node = None
    try:
        node = Rosbag2RecorderNode(namespace=args.namespace)
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
