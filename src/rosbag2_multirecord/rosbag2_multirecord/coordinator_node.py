import os
import shutil
import subprocess
import threading
import time
import traceback
from pathlib import Path

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

NODE_NAME: str = "multirecord_coordinator"


class MultiRecordCoordinator(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        # create a reentrant callback group for service clients
        # s.t. they can execute concurrently with service callbacks
        self.client_callback_group = ReentrantCallbackGroup()

        self.declare_parameter("output_bag_dir", "")
        self.declare_parameter(
            "recorder_namespaces", [""]
        )  # default to array of empty string for type inference
        self.declare_parameter("recorder_discovery_timeout", 10.0)

        self.output_bag_dir = self.get_parameter("output_bag_dir").value
        self.recorder_namespaces = self.get_parameter("recorder_namespaces").value
        self.discovery_timeout = self.get_parameter("recorder_discovery_timeout").value

        self.num_recorders = len(self.recorder_namespaces)
        self.started = False

        # get path to mcap binary
        pkg_dir = Path(__file__).parent
        self.mcap_bin = pkg_dir / "bin" / "mcap"

        # validate parameters
        if not self.output_bag_dir:
            self.get_logger().error("output_bag_dir parameter is required")
            raise ValueError("output_bag_dir is required")

        if self.num_recorders <= 0:
            self.get_logger().error("recorder_namespaces cannot be empty")
            raise ValueError("recorder_namespaces is empty")

        # check if output directory already exists
        if os.path.exists(self.output_bag_dir):
            self.get_logger().error(
                f"Output directory already exists: {self.output_bag_dir}"
            )
            raise ValueError(f"Output directory exists: {self.output_bag_dir}")

        self.get_logger().info(f"Output bag directory: {self.output_bag_dir}")
        self.get_logger().info(f"Number of recorders: {self.num_recorders}")
        self.get_logger().info(f"Recorder namespaces: {self.recorder_namespaces}")

        # service clients for recorder nodes
        self.recorder_info = []

        # discover and connect to all recorder nodes
        if not self._discover_recorders():
            self.get_logger().error("Failed to discover all recorder nodes")
            raise RuntimeError("Recorder discovery failed")

        # create coordinator services
        self.start_srv = self.create_service(
            Trigger, "~/start", self.start_recording_callback
        )
        self.pause_srv = self.create_service(
            Trigger, "~/pause", self.pause_recording_callback
        )
        self.stop_srv = self.create_service(
            Trigger, "~/stop", self.stop_recording_callback
        )

        self.shutdown_timer = None

        self.get_logger().info("Coordinator ready.")

    def _call_recorder_services(self, service_name, client_attr, timeout=5.0):
        futures = []
        for info in self.recorder_info:
            future = info[client_attr].call_async(Trigger.Request())
            futures.append((info, future))

        # wait for all responses
        start_time = time.time()
        successful_recorders = []

        for info, future in futures:
            remaining = timeout - (time.time() - start_time)
            if remaining <= 0:
                self.get_logger().error(
                    f"Recorder {info['node_name']} {service_name} service call timed out"
                )
                continue

            # wait for future to complete
            end_time = time.time() + remaining
            while not future.done() and time.time() < end_time:
                time.sleep(0.01)

            if not future.done():
                self.get_logger().error(
                    f"Recorder {info['node_name']} {service_name} service call timed out"
                )
            else:
                try:
                    result = future.result()
                    if not result.success:
                        self.get_logger().error(
                            f"Recorder {info['node_name']} failed to {service_name}: {result.message}"
                        )
                    else:
                        self.get_logger().info(
                            f"Recorder {info['node_name']} {service_name}ed successfully"
                        )
                        successful_recorders.append(info)
                except Exception as e:
                    self.get_logger().error(
                        f"Recorder {info['node_name']} {service_name} service call failed: {e}"
                    )

        all_success = len(successful_recorders) == len(self.recorder_info)
        return all_success, successful_recorders

    def _get_recorder_node_name(self, namespace):
        if namespace == "":
            return "/rosbag2_recorder"
        else:
            return f"/rosbag2_recorder_{namespace}"

    def _discover_recorders(self):
        self.get_logger().info("Discovering recorder nodes...")

        start_time = time.time()

        for namespace in self.recorder_namespaces:
            recorder_node = self._get_recorder_node_name(namespace)

            # create service clients with reentrant callback group
            start_client = self.create_client(
                Trigger,
                f"{recorder_node}/start",
                callback_group=self.client_callback_group,
            )
            pause_client = self.create_client(
                Trigger,
                f"{recorder_node}/pause",
                callback_group=self.client_callback_group,
            )
            stop_client = self.create_client(
                Trigger,
                f"{recorder_node}/stop",
                callback_group=self.client_callback_group,
            )

            param_client = self.create_client(
                GetParameters,
                f"{recorder_node}/get_parameters",
                callback_group=self.client_callback_group,
            )

            # wait for services to be available
            timeout_remaining = self.discovery_timeout - (time.time() - start_time)
            if timeout_remaining <= 0:
                self.get_logger().error(f"Timeout discovering recorder {recorder_node}")
                return False

            if not start_client.wait_for_service(timeout_sec=timeout_remaining):
                self.get_logger().error(
                    f"Recorder {recorder_node} start service not available after {timeout_remaining}s"
                )
                return False

            if not pause_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error(
                    f"Recorder {recorder_node} pause service not available"
                )
                return False

            if not stop_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error(
                    f"Recorder {recorder_node} stop service not available"
                )
                return False

            if not param_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error(
                    f"Recorder {recorder_node} parameter service not available"
                )
                return False

            # get output_dir parameter from recorder
            param_request = GetParameters.Request()
            param_request.names = ["output_dir"]
            param_future = param_client.call_async(param_request)

            # wait for parameter response
            rclpy.spin_until_future_complete(self, param_future, timeout_sec=5.0)

            if param_future.result() is None:
                self.get_logger().error(
                    f"Failed to get output_dir from {recorder_node}"
                )
                return False

            param_values = param_future.result().values
            if not param_values or param_values[0].type == 0:  # type 0 = not set
                self.get_logger().error(
                    f"output_dir parameter not set on {recorder_node}"
                )
                return False

            output_dir = param_values[0].string_value

            self.recorder_info.append(
                {
                    "namespace": namespace,
                    "node_name": recorder_node,
                    "output_dir": output_dir,
                    "start_client": start_client,
                    "pause_client": pause_client,
                    "stop_client": stop_client,
                }
            )

            # self.get_logger().info(
            #     f"Recorder {recorder_node} discovered (output: {output_dir})"
            # )

        lines = "\n".join(
            f"\t{i['node_name']} (output: {i['output_dir']})"
            for i in self.recorder_info
        )
        self.get_logger().info(
            f"All recorders discovered ({self.num_recorders}):\n{lines}"
        )

        return True

    def _shutdown(self):
        if self.shutdown_timer:
            self.shutdown_timer.cancel()

        threading.Thread(target=rclpy.shutdown, daemon=True).start()

    def start_recording_callback(self, request, response):
        self.get_logger().info("Starting recording on all recorders...")

        all_success, _ = self._call_recorder_services(
            "start", "start_client", timeout=5.0
        )

        if all_success:
            response.success = True
            response.message = "All recorders started successfully"
            self.get_logger().info("All recorders started")

            self.started = True
        else:
            response.success = False
            response.message = "Some recorders failed to start"

        return response

    def pause_recording_callback(self, request, response):
        self.get_logger().info("Pausing recording on all recorders...")

        all_success, _ = self._call_recorder_services(
            "pause", "pause_client", timeout=5.0
        )

        if all_success:
            response.success = True
            response.message = "All recorders paused successfully"
            self.get_logger().info("All recorders paused")
        else:
            response.success = False
            response.message = "Some recorders failed to pause"

        return response

    def stop_recording_callback(self, request, response):
        if not self.started:
            response.success = False
            response.message = "Recording has not been started"
            return response

        self.get_logger().info("Stopping recording on all recorders...")

        _, successful_recorders = self._call_recorder_services(
            "stop", "stop_client", timeout=10.0
        )

        if not successful_recorders:
            response.success = False
            response.message = "All recorders failed to stop"
            return response

        # merge bags from successful recorders
        try:
            self._merge_bags(successful_recorders)
            self._cleanup_temp_dirs(successful_recorders)

            response.success = True
            response.message = "Recording stopped, bags merged successfully"
            self.get_logger().info("Recording completed successfully")
        except Exception as e:
            self.get_logger().error(f"Error during merge/cleanup: {e}")
            response.success = False
            response.message = f"Merge failed: {str(e)}"

        # schedule shutdown after this callback returns
        self.shutdown_timer = self.create_timer(
            0.0,  # next spin
            self._shutdown,
        )

        return response

    def _merge_bags(self, successful_recorders):
        """Merge bags from successful recorders"""
        self.get_logger().info("Merging bags...")

        # collect all MCAP files from successful recorders
        mcap_files = []
        for info in successful_recorders:
            recorder_dir = info["output_dir"]
            mcap_pattern = Path(recorder_dir).glob("*.mcap")
            mcap_list = list(mcap_pattern)

            if not mcap_list:
                self.get_logger().warn(f"No MCAP files found in {recorder_dir}")
                continue

            mcap_files.extend([str(f) for f in mcap_list])

        if not mcap_files:
            raise RuntimeError("No MCAP files found to merge")

        self.get_logger().info(f"Found {len(mcap_files)} MCAP files to merge")

        # create output directory
        os.makedirs(self.output_bag_dir, exist_ok=True)

        # determine output MCAP filename
        bag_name = Path(self.output_bag_dir).name
        merged_mcap = os.path.join(self.output_bag_dir, f"{bag_name}_0.mcap")

        # run mcap merge
        merge_cmd = [
            str(self.mcap_bin),
            "merge",
            "-o",
            merged_mcap,
            "--allow-duplicate-metadata",
        ] + mcap_files

        self.get_logger().info(f"Running: {' '.join(merge_cmd)}")
        result = subprocess.run(merge_cmd, capture_output=True, text=True)

        if result.returncode != 0:
            raise RuntimeError(f"mcap merge failed: {result.stderr}")

        self.get_logger().info(f"Merged bag created: {merged_mcap}")

        # reindex the merged bag
        self.get_logger().info("Reindexing merged bag...")
        reindex_result = subprocess.run(
            ["ros2", "bag", "reindex", "-s", "mcap", self.output_bag_dir],
            capture_output=True,
            text=True,
        )

        if reindex_result.returncode != 0:
            self.get_logger().warn(f"Reindex failed: {reindex_result.stderr}")
        else:
            self.get_logger().info("Merged bag reindexed successfully")

    def _cleanup_temp_dirs(self, successful_recorders):
        self.get_logger().info("Cleaning up temporary directories...")

        for info in successful_recorders:
            recorder_dir = info["output_dir"]
            if os.path.exists(recorder_dir):
                try:
                    shutil.rmtree(recorder_dir)
                    self.get_logger().info(f"Removed {recorder_dir}")
                except Exception as e:
                    self.get_logger().warn(f"Failed to remove {recorder_dir}: {e}")


def main(args=None):
    rclpy.init(args=args)

    logger = rclpy.logging.get_logger(NODE_NAME)

    node = None
    try:
        executor = MultiThreadedExecutor()
        node = MultiRecordCoordinator()
        executor.add_node(node)
        executor.spin()
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
