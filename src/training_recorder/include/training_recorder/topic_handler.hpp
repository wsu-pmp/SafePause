
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <rclcpp/serialization.hpp>
#include <string>
#include <vector>

namespace training_recorder {
class TopicHandlerBase;
template <typename T> class TopicHandler;

// Small fixed-size buffer that holds serialized bytes
struct SerializedBuffer {
  uint8_t *data;
  std::size_t size;     // actual occupied size
  std::size_t capacity; // total capacity
  int64_t timestamp;    // nanoseconds
  std::string topic_name;
};

// Lightweight profiler counters (very low overhead)
struct ProfilerCounters {
  std::atomic<uint64_t> received{0};
  std::atomic<uint64_t> enqueued{0};
  std::atomic<uint64_t> dropped{0};
  std::atomic<uint64_t> written{0};
};

class BufferPool {
public:
  BufferPool(size_t buf_size, size_t pool_size);
  ~BufferPool();

  uint8_t *acquire();
  void release(uint8_t *p);
  size_t buffer_capacity() const;

private:
  size_t buf_size_;
  std::mutex m_;
  std::vector<uint8_t *> free_list_;
  std::vector<uint8_t *> leased_; // for cleanup
};

// Lock-protected queue of SerializedBuffer shared pointers
class MessageQueue {
public:
  MessageQueue(size_t capacity);

  bool push(std::shared_ptr<SerializedBuffer> msg);
  std::shared_ptr<SerializedBuffer>
  pop_or_wait(bool &shutdown_flag); // blocking pop with shutdown awareness
  std::shared_ptr<SerializedBuffer> try_pop(); // attempt non-blocking pop
  size_t size() const;

private:
  mutable std::mutex m_;
  std::condition_variable cv_;
  std::deque<std::shared_ptr<SerializedBuffer>> queue_;
  size_t capacity_;
};

class TopicHandlerFactoryBase {
public:
  virtual ~TopicHandlerFactoryBase() = default;
  virtual const std::string &topic_name() const = 0;
  virtual std::shared_ptr<TopicHandlerBase>
  create(rclcpp::Node::SharedPtr parent, size_t queue_capacity,
         BufferPool *pool, std::function<void()> notify_cb) const = 0;
};

template <typename T>
class TopicHandlerFactory : public TopicHandlerFactoryBase {
public:
  TopicHandlerFactory(const std::string &topic_name)
      : topic_name_(topic_name) {}

  const std::string &topic_name() const override { return this->topic_name_; }
  std::shared_ptr<TopicHandlerBase>
  create(rclcpp::Node::SharedPtr parent, size_t queue_capacity,
         BufferPool *pool, std::function<void()> notify_cb) const override {
    return std::make_shared<TopicHandler<T>>(parent, this->topic_name_,
                                             queue_capacity, pool, notify_cb);
  }

private:
  std::string topic_name_;
};

// Base class for topic handlers to allow polymorphic storage
class TopicHandlerBase {
public:
  virtual ~TopicHandlerBase() = default;
  virtual void init() = 0;
  virtual const std::string &topic_name() const = 0;
  virtual const std::string &topic_type() const = 0;
  virtual MessageQueue &queue() = 0;
  virtual BufferPool *pool() = 0;
  virtual ProfilerCounters &profiler() = 0;
};

// The per-topic handler -- templated for known message type
template <typename T> class TopicHandler : public TopicHandlerBase {
public:
  TopicHandler(rclcpp::Node::SharedPtr parent, const std::string &topic_name,
               size_t queue_capacity, BufferPool *pool,
               std::function<void()> notify_cb)
      : parent_(parent), topic_name_(topic_name),
        topic_type_(rosidl_generator_traits::name<T>()), queue_(queue_capacity),
        pool_(pool), notify_cb_(notify_cb) {}

  void init() override {
    sub_ = this->parent_->template create_subscription<T>(
        topic_name_, rclcpp::QoS(rclcpp::KeepLast(100)),
        [this](const typename T::SharedPtr msg) { this->callback(msg); });
  }

  void callback(const typename T::SharedPtr &msg) {
    profiler_.received.fetch_add(1, std::memory_order_relaxed);

    uint8_t *buffer = pool_->acquire();
    size_t capacity = pool_->buffer_capacity();

    rclcpp::SerializedMessage serialized_msg;
    rclcpp::Serialization<T> serializer;
    serializer.serialize_message(msg.get(), &serialized_msg);

    if (serialized_msg.size() > capacity) {
      profiler_.dropped.fetch_add(1, std::memory_order_relaxed);
      pool_->release(buffer);
      return;
    }

    memcpy(buffer, serialized_msg.get_rcl_serialized_message().buffer,
           serialized_msg.size());

    auto sb = std::make_shared<SerializedBuffer>();
    sb->data = buffer;
    sb->size = serialized_msg.size();
    sb->capacity = capacity;
    sb->timestamp = parent_->get_clock()->now().nanoseconds();
    sb->topic_name = topic_name_;

    bool ok = queue_.push(sb);
    if (!ok) {
      profiler_.dropped.fetch_add(1, std::memory_order_relaxed);
      pool_->release(buffer);
      return;
    }

    profiler_.enqueued.fetch_add(1, std::memory_order_relaxed);

    notify_cb_();
  }

  MessageQueue &queue() override { return queue_; }
  BufferPool *pool() override { return pool_; }
  ProfilerCounters &profiler() override { return profiler_; }
  const std::string &topic_name() const override { return topic_name_; }
  const std::string &topic_type() const override { return topic_type_; }

private:
  rclcpp::Node::SharedPtr parent_;
  std::string topic_name_;
  std::string topic_type_;
  typename rclcpp::Subscription<T>::SharedPtr sub_;
  MessageQueue queue_;
  BufferPool *pool_;
  ProfilerCounters profiler_;
  std::function<void()> notify_cb_;
};
} // namespace training_recorder
