#pragma once

// #include <rclcpp/executors/multi_threaded_executor.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
// #include <rclcpp/serialization.hpp>
// #include <rclcpp/serialized_message.hpp>
// #include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
// #include <rosbag2_storage/serialized_bag_message.hpp>
// #include <rosbag2_storage/storage_options.hpp>
// #include <rosbag2_storage/topic_metadata.hpp>
// #include <rosidl_runtime_cpp/traits.hpp>
// // #include <std_srvs/srv/trigger.hpp>

// #include <sensor_msgs/msg/detail/joint_state__struct.hpp>
// #include <sensor_msgs/msg/joint_state.hpp>
// #include <std_msgs/msg/string.hpp>

// #include <atomic>
// #include <chrono>
#include <condition_variable>
// #include <deque>
// #include <memory>
#include <mutex>
// #include <sstream>
// #include <thread>
#include <string>
#include <vector>

#include "training_recorder/topic_handler.hpp"

namespace training_recorder {
class RecorderNode : public rclcpp::Node {
public:
  RecorderNode(
      std::vector<std::unique_ptr<TopicHandlerFactoryBase>> topic_factories,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  // prepare resources but defer recording until start(path) is called
  bool init();

  // start recording to storage path - spin executor and start writer thread
  std::pair<bool, std::string> start(const std::string &storage_path);

  // stop topic cb exec, drain message queues, close writer
  std::pair<bool, std::string> stop();

  // profiler stats
  void get_profiler_snapshot();

private:
  // topic callbacks
  std::vector<std::unique_ptr<TopicHandlerFactoryBase>> topic_factories_;
  std::unique_ptr<BufferPool> buffer_pool_;
  std::vector<std::shared_ptr<TopicHandlerBase>> topic_handlers_;
  int num_callback_threads_;

  // threading
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread executor_thread_;
  std::thread writer_thread_;

  // control flags
  std::atomic<bool> is_init_;
  std::atomic<bool> running_;
  std::atomic<bool> shutting_down_;

  // writer
  std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;
  std::string storage_path_;
  std::atomic<uint64_t> pending_writes_{0};
  std::mutex writer_mutex_;
  std::condition_variable writer_wake_;
  std::optional<std::exception_ptr> writer_exception_;
  size_t round_robin_index_;

  bool write_one();
  void writer_loop();
};
} // namespace training_recorder