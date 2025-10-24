#pragma once

#include <event_logger/event_logger.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/node.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <safepause_msgs/msg/message_bundle_index.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace perception_pkg {

struct TopicConfig {
  std::string name;
  std::string type;
  bool requires_tf{false};
  std::string target_frame;
  double max_tf_age{0.0};
};

struct DeserializedMessage {
  std::shared_ptr<void> msg;
  const rosidl_message_type_support_t *type_support;
  const rosidl_message_type_support_t *introspection_ts;

  rclcpp::Time stamp;
  std::string frame_id;
  bool has_header{false};
};

struct MessageEnvelope {
  std::string topic_name;
  std::string type_string;
  DeserializedMessage data;

  bool has_transform{false};
  geometry_msgs::msg::TransformStamped transform;
};

struct MessageBundle {
  rclcpp::Time bundle_time;
  std::map<std::string, MessageEnvelope> entries;
};

class PerceptionNode final : public rclcpp::Node {
public:
  PerceptionNode(const std::string &ns);
  ~PerceptionNode();

  static std::string get_namespace_append(const std::string &ns);

private:
  void load_config();
  void create_subscriptions();

  void discovery_timer_callback();
  std::pair<bool, std::vector<std::pair<std::string, std::string>>>
  validate_discovery();

  const rosidl_message_type_support_t *get_ts(const std::string &type_str);
  const rosidl_message_type_support_t *
  get_introspection_ts(const std::string &type_str);
  bool extract_header(const DeserializedMessage &dm, rclcpp::Time &stamp,
                      std::string &frame_id);

  void generic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg,
                        std::size_t topic_index);

  // caller must hold sync_mutex_
  // returns bundles for the caller to publish once the lock is released
  std::vector<MessageBundle> try_create_bundle();
  std::optional<geometry_msgs::msg::TransformStamped>
  resolve_transform(const std::string &target_frame,
                    const std::string &source_frame, const rclcpp::Time &time,
                    double max_age);

  void publish_bundle_index(const MessageBundle &bundle);
  void enqueue_bundle(MessageBundle bundle);

  void processing_thread_main();
  void process_bundle(const MessageBundle &bundle);

  template <typename T>
  static const T *try_cast(const MessageEnvelope &env,
                           const std::string &topic_name,
                           const std::string &expect_type) {
    // check type string matches expected before downcast
    // validate_discovery() ensures env.type_string matches discovered topic
    if (env.type_string != expect_type) {
      throw std::runtime_error(
          "Bundle processing encountered unexpected message type for '" +
          topic_name + "': expected '" + expect_type + "', got '" +
          env.type_string + "'");
    }

    // downcast message to its original type
    return static_cast<const T *>(env.data.msg.get());
  }

  std::vector<TopicConfig> topic_configs_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  rclcpp::Publisher<safepause_msgs::msg::MessageBundleIndex>::SharedPtr
      index_publisher_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::vector<std::deque<MessageEnvelope>> message_queues_;
  std::mutex sync_mutex_;

  std::size_t queue_size_{10};
  double slop_{0.1};
  std::atomic<uint64_t> dropped_messages_count_{0};
  std::atomic<uint64_t> dropped_bundles_count_{0};

  rclcpp::TimerBase::SharedPtr discovery_timer_;
  rclcpp::Time discovery_start_time_;

  std::deque<MessageBundle> processing_queue_;
  std::mutex processing_mutex_;
  std::condition_variable processing_cv_;
  std::thread processing_thread_;
  std::atomic<bool> running_{true};
  double processing_rate_{10.0};
  std::size_t processing_queue_size_{100};

  std::unique_ptr<event_logger::EventLogger> events_;

  // topics currently publishing without their required transform
  std::set<std::string> degraded_topics_;

  // counters last reported
  uint64_t reported_dropped_messages_{0};
  uint64_t reported_dropped_bundles_{0};
  uint64_t bundles_published_{0};
  uint64_t reported_bundles_published_{0};
  rclcpp::TimerBase::SharedPtr summary_timer_;
  void log_summary();
};

} // namespace perception_pkg