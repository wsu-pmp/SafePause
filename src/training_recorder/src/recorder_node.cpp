#include <exception>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
// #include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
// #include <rosbag2_storage/serialized_bag_message.hpp>
// #include <rosbag2_storage/storage_options.hpp>
// #include <rosbag2_storage/topic_metadata.hpp>
// #include <rosidl_runtime_cpp/traits.hpp>

// #include <sensor_msgs/msg/detail/joint_state__struct.hpp>
// #include <sensor_msgs/msg/joint_state.hpp>
// #include <std_msgs/msg/string.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

#include "training_recorder/recorder_node.hpp"

namespace training_recorder {
RecorderNode::RecorderNode(
    std::vector<std::unique_ptr<TopicHandlerFactoryBase>> topic_factories,
    const rclcpp::NodeOptions &options)
    : rclcpp::Node("recorder", options),
      topic_factories_(std::move(topic_factories)), is_init_(false),
      running_(false), shutting_down_(false), round_robin_index_(0) {

  writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();

  // num_callback_threads default based on hardware concurrency
  size_t num_hw_cores = std::thread::hardware_concurrency();
  if (num_hw_cores == 0)
    num_hw_cores = 4;

  this->declare_parameter(
      "num_callback_threads",
      static_cast<int>(std::max<size_t>(2, num_hw_cores - 1)));
  this->declare_parameter("queue_capacity", 1024);
  this->declare_parameter("buffer_size_bytes",
                          2 * 1024 * 1024); // 2 MB default
  this->declare_parameter("pool_size", 64);
}

// prepare resources but defer recording until start(path) is called
bool RecorderNode::init() {
  if (this->is_init_.load()) {
    return false;
  }

  int num_cb_threads = this->get_parameter("num_callback_threads").as_int();
  size_t queue_capacity =
      static_cast<size_t>(this->get_parameter("queue_capacity").as_int());
  size_t buffer_size =
      static_cast<size_t>(this->get_parameter("buffer_size_bytes").as_int());
  size_t pool_size =
      static_cast<size_t>(this->get_parameter("pool_size").as_int());

  RCLCPP_INFO(this->get_logger(), "Initialised:");
  RCLCPP_INFO(this->get_logger(), "\tnum_callback_threads: %d", num_cb_threads);
  RCLCPP_INFO(this->get_logger(), "\tqueue_capacity: %zu", queue_capacity);
  RCLCPP_INFO(this->get_logger(), "\tbuffer_size_bytes: %zu", buffer_size);
  RCLCPP_INFO(this->get_logger(), "\tpool_size: %zu", buffer_size);

  buffer_pool_ = std::make_unique<BufferPool>(buffer_size, pool_size);

  auto notify_cb = [this]() {
    pending_writes_.fetch_add(1, std::memory_order_release);
    writer_wake_.notify_one();
  };

  // create TopicHandler objects
  for (auto &f : this->topic_factories_) {
    auto th = f->create(shared_from_this(), queue_capacity, buffer_pool_.get(),
                        notify_cb);
    this->topic_handlers_.push_back(th);
  }

  // build executor
  num_callback_threads_ = std::max<int>(1, num_cb_threads);

  // init topic subscriptions for each handler
  for (auto &th : topic_handlers_)
    th->init();

  this->is_init_.store(true);
  return true;
}

// spin the executor and start writer thread.
std::pair<bool, std::string>
RecorderNode::start(const std::string &storage_path) {
  storage_path_ = std::filesystem::weakly_canonical(storage_path);

  // create mcap writer
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = storage_path_;
  storage_options.storage_id = "mcap";

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  try {
    writer_->open(storage_options, converter_options);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open bag: %s", e.what());

    return {false, std::string("Failed to open bag: ") + e.what()};
  }

  // reset stats
  for (auto &th : topic_handlers_) {
    th->profiler().received.store(0);
    th->profiler().enqueued.store(0);
    th->profiler().dropped.store(0);
    th->profiler().written.store(0);
  }

  if (running_.load()) {
    return {false, "Recorder is running"};
  }

  if (!this->is_init_.load()) {
    return {false, "Recorder has not been initialised"};
  }

  shutting_down_.store(false);

  // start the executor with num_callback_threads_ threads
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), static_cast<size_t>(num_callback_threads_));

  executor_->add_node(shared_from_this());

  executor_thread_ = std::thread([this]() { executor_->spin(); });

  // start writer thread
  this->writer_thread_ = std::thread(&RecorderNode::writer_loop, this);

  running_.store(true);
  RCLCPP_INFO(this->get_logger(), "Recorder started, writing to: %s",
              storage_path_.c_str());

  return {true, "Recorder started"};
}

// graceful close: stop accepting new topics, flush queues, stop writers
std::pair<bool, std::string> RecorderNode::stop() {
  if (!running_.load()) {
    return {false, "Recorder is not running"};
  }

  RCLCPP_INFO(this->get_logger(), "Stop requested");

  // stop taking new messages by shutting down executor
  shutting_down_.store(true);
  writer_wake_.notify_all();

  // ask executor to cancel and stop
  if (executor_) {
    executor_->cancel();
  }

  // join executor thread
  if (executor_thread_.joinable())
    executor_thread_.join();

  // wait for writer thread to drain queues and exit
  if (writer_thread_.joinable()) {
    writer_thread_.join();
  }

  writer_->close();

  running_.store(false);

  if (writer_exception_.has_value()) {
    try {
      std::rethrow_exception(writer_exception_.value());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "%s", e.what());

      return {false, e.what()};
    }

  } else {
    RCLCPP_INFO(this->get_logger(), "Recorder stopped");

    return {true, "Recorder stopped"};
  }
}

// lightweight endpoint to fetch profiler statistics
void RecorderNode::get_profiler_snapshot() {
  for (auto &th : topic_handlers_) {
    RCLCPP_INFO(this->get_logger(),
                "Topic %s: received=%lu enqueued=%lu dropped=%lu written=%lu "
                "queue_size=%zu",
                th->topic_name().c_str(), th->profiler().received.load(),
                th->profiler().enqueued.load(), th->profiler().dropped.load(),
                th->profiler().written.load(), th->queue().size());
  }
}

bool RecorderNode::write_one() {
  // iterate over all topic handlers round-robin
  size_t n = this->topic_handlers_.size();
  for (size_t i = 0; i < n; ++i) {
    std::shared_ptr<TopicHandlerBase> th =
        this->topic_handlers_.at(round_robin_index_);
    round_robin_index_ = (round_robin_index_ + 1) % n;

    auto &q = th->queue();
    auto item = q.try_pop();
    if (!item)
      continue; // queue empty, move to next TopicHandler

    try {
      // build the SerializedBagMessage re-using the pooled buffer
      auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
      bag_msg->time_stamp = item->timestamp;
      bag_msg->topic_name = item->topic_name;

      // rcutils_uint8_array_t with our SerializedBuffer
      auto arr = new rcutils_uint8_array_t();
      arr->buffer = item->data; // borrowed from pool
      arr->buffer_length = item->size;
      arr->buffer_capacity = item->capacity;
      arr->allocator = rcutils_get_default_allocator();

      // shared_ptr with custom delete that returns buffer to pool.
      BufferPool *pool = th->pool();
      bag_msg->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
          arr, [pool](rcutils_uint8_array_t *p) {
            // return the raw buffer to the pool and delete the rcutils struct.
            if (p->buffer) {
              pool->release(p->buffer);
              p->buffer = nullptr;
            }
            delete p;
          });

      // write
      try {
        writer_->write(bag_msg);
        th->profiler().written.fetch_add(1, std::memory_order_relaxed);
      } catch (const std::exception &e) {
        // If write fails, ensure buffer is returned by resetting shared_ptr
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to write message for topic %s: %s",
                     bag_msg->topic_name.c_str(), e.what());
        // release will happen when bag_msg goes out of scope / shared_ptr
        // destroyed

        writer_exception_ = std::current_exception();
        return false; // signal failure to writer_loop()
      }

    } catch (const std::exception &e) {
      writer_exception_ = std::current_exception();
      return false; // signal failure to writer_loop()
    }

    return true;
  }

  return false; // nothing to write in any queue
}

void RecorderNode::writer_loop() {
  // register topics
  for (auto &th : topic_handlers_) {
    rosbag2_storage::TopicMetadata meta;
    meta.name = th->topic_name();
    meta.type = th->topic_type();
    meta.serialization_format = "cdr";

    try {
      writer_->create_topic(meta);
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "create_topic('%s') failed: %s",
                  meta.name.c_str(), e.what());
    }
  }

  std::unique_lock<std::mutex> lk(writer_mutex_);

  while (!shutting_down_.load(std::memory_order_acquire)) {
    // Sleep until there's work
    writer_wake_.wait(lk, [&] {
      return shutting_down_.load(std::memory_order_acquire) ||
             (pending_writes_.load(std::memory_order_acquire) > 0);
    });

    if (shutting_down_.load(std::memory_order_acquire))
      break;

    // release lock during writes
    lk.unlock();

    // keep draining while there is writes remaining
    while (pending_writes_.load(std::memory_order_acquire) > 0) {

      if (!write_one()) {
        // either no work (pending_writes_ > 0 but nothing in queue) or failure
        // occurred
        if (writer_exception_.has_value()) {
          shutting_down_.store(true);
        }
        break;
      }

      pending_writes_.fetch_sub(1);
    }

    // reacquire lock before re-waiting
    lk.lock();
  }
}
} // namespace training_recorder