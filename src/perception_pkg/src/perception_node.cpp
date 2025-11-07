#include <exception>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/typesupport_helpers.hpp>
#include <rclcpp/utilities.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <sstream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

#include "perception_pkg/perception_node.hpp"

namespace perception_pkg {

const std::string NODE_NAME = "perception_node";

PerceptionNode::PerceptionNode(const std::string &ns)
    : Node(NODE_NAME + PerceptionNode::get_namespace_append(ns)) {
  declare_parameter("config_file", "");
  declare_parameter("queue_size", 10);
  declare_parameter("slop", 0.1);
  declare_parameter("processing_rate", 10.0);

  queue_size_ = get_parameter("queue_size").as_int();
  slop_ = get_parameter("slop").as_double();
  processing_rate_ = get_parameter("processing_rate").as_double();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  std::string topic_name =
      "~/bundle_index" + PerceptionNode::get_namespace_append(ns);
  index_publisher_ =
      create_publisher<safepause_msgs::msg::MessageBundleIndex>(topic_name, 10);

  load_config();

  // defer subscription(s) until all topics are found
  discovery_start_time_ = now();
  discovery_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&PerceptionNode::discovery_timer_callback, this));
}

PerceptionNode::~PerceptionNode() {
  running_ = false;
  processing_cv_.notify_all();
  if (processing_thread_.joinable())
    processing_thread_.join();
}

void PerceptionNode::discovery_timer_callback() {
  const auto [valid, topics] = validate_discovery();
  if (valid) {
    std::ostringstream oss;
    oss << "Discovered topic" << (topics.size() != 1 ? "s" : "") << ":\n";
    for (auto topic : topics) {
      oss << "\t" << topic.first << " (" << topic.second << ")\n";
    }
    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());

    create_subscriptions();
    processing_thread_ =
        std::thread(&PerceptionNode::processing_thread_main, this);

    discovery_timer_->cancel();
    return;
  }

  auto elapsed = (now() - discovery_start_time_).seconds();
  if (elapsed >= 10.0) {
    std::ostringstream oss;
    oss << "Unable to discover topic" << (topics.size() != 1 ? "s" : "")
        << ":\n";
    for (auto topic : topics) {
      oss << "\t" << topic.first << " (" << topic.second << ")\n";
    }
    RCLCPP_ERROR(get_logger(), "%s", oss.str().c_str());

    discovery_timer_->cancel();
    rclcpp::shutdown();
  } else {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
                         "Waiting for topics...");
  }
}

std::pair<bool, std::vector<std::pair<std::string, std::string>>>
PerceptionNode::validate_discovery() {
  const std::map<std::string, std::vector<std::string>> network_topics =
      this->get_topic_names_and_types();

  bool all_found = true;
  std::vector<std::pair<std::string, std::string>> all_topics;
  std::vector<std::pair<std::string, std::string>> failed_topics;

  for (const auto &cfg : topic_configs_) {
    const std::string full_name =
        this->get_node_topics_interface()->resolve_topic_name(cfg.name);

    bool found = true;

    // topic exists
    auto it = network_topics.find(full_name);
    if (it == network_topics.end()) {
      found = false;
    }

    // topic has at least one external publisher
    if (found && this->count_publishers(full_name) == 0) {
      found = false;
    }

    // type compatibility
    if (found) {
      bool type_match = false;
      for (const auto &discovered_type : it->second) {
        if (discovered_type == cfg.type ||
            discovered_type ==
                cfg.type.substr(0, cfg.type.find_last_of('/')) + "/msg/" +
                    cfg.type.substr(cfg.type.find_last_of('/') + 1)) {
          type_match = true;
          break;
        }
      }

      if (!type_match) {
        found = false;
      }
    }

    all_topics.emplace_back(full_name, cfg.type);

    if (!found) {
      all_found = false;
      failed_topics.emplace_back(full_name, cfg.type);
    }
  }

  if (all_found) {
    return {true, all_topics};
  }

  return {false, failed_topics};
}

void PerceptionNode::load_config() {
  const auto config_file = get_parameter("config_file").as_string();
  if (config_file.empty()) {
    throw std::runtime_error("'config_file' parameter must be provided");
  }

  const YAML::Node root = YAML::LoadFile(config_file);
  for (const auto &node : root["topics"]) {
    TopicConfig cfg;
    cfg.name = node["name"].as<std::string>();
    cfg.type = node["type"].as<std::string>();
    cfg.requires_tf = node["requires_tf"].as<bool>(false);
    cfg.target_frame = node["target_frame"].as<std::string>("");
    cfg.max_tf_age = node["max_tf_age"].as<double>(0.1);
    topic_configs_.push_back(cfg);
  }
  message_queues_.resize(topic_configs_.size());
}

const rosidl_message_type_support_t *
PerceptionNode::get_ts(const std::string &type_str) {
  return rclcpp::get_typesupport_handle(
      type_str, "rosidl_typesupport_cpp",
      *rclcpp::get_typesupport_library(type_str, "rosidl_typesupport_cpp"));
}

const rosidl_message_type_support_t *
PerceptionNode::get_introspection_ts(const std::string &type_str) {
  return rclcpp::get_typesupport_handle(
      type_str, "rosidl_typesupport_introspection_cpp",
      *rclcpp::get_typesupport_library(type_str,
                                       "rosidl_typesupport_introspection_cpp"));
}

bool PerceptionNode::extract_header(const DeserializedMessage &dm,
                                    rclcpp::Time &stamp,
                                    std::string &frame_id) {
  auto members =
      static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
          dm.introspection_ts->data);
  for (uint32_t i = 0; i < members->member_count_; ++i) {
    const auto &member = members->members_[i];
    if (std::string(member.name_) == "header") {
      const void *header_ptr =
          static_cast<const uint8_t *>(dm.msg.get()) + member.offset_;
      auto header_members = static_cast<
          const rosidl_typesupport_introspection_cpp::MessageMembers *>(
          member.members_->data);

      for (uint32_t j = 0; j < header_members->member_count_; ++j) {
        const auto &h_member = header_members->members_[j];
        const void *field_ptr =
            static_cast<const uint8_t *>(header_ptr) + h_member.offset_;

        if (std::string(h_member.name_) == "stamp") {
          stamp = rclcpp::Time(
              *static_cast<const builtin_interfaces::msg::Time *>(field_ptr));
        } else if (std::string(h_member.name_) == "frame_id") {
          frame_id = *static_cast<const std::string *>(field_ptr);
        }
      }

      return true;
    }
  }
  return false;
}

void PerceptionNode::create_subscriptions() {
  for (std::size_t i = 0; i < topic_configs_.size(); ++i) {
    const auto &cfg = topic_configs_[i];
    subscriptions_.push_back(create_generic_subscription(
        cfg.name, cfg.type, rclcpp::SystemDefaultsQoS(),
        [this, i](std::shared_ptr<rclcpp::SerializedMessage> msg) {
          generic_callback(msg, i);
        }));
  }
}

void PerceptionNode::generic_callback(
    std::shared_ptr<rclcpp::SerializedMessage> msg, std::size_t topic_index) {
  const auto &cfg = topic_configs_[topic_index];
  DeserializedMessage dm;
  dm.type_support = get_ts(cfg.type);
  dm.introspection_ts = get_introspection_ts(cfg.type);

  auto members =
      static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
          dm.introspection_ts->data);
  dm.msg = std::shared_ptr<void>(malloc(members->size_of_), free);
  members->init_function(dm.msg.get(),
                         rosidl_runtime_cpp::MessageInitialization::ALL);

  try {
    rclcpp::SerializationBase(dm.type_support)
        .deserialize_message(msg.get(), dm.msg.get());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "deserialization failed for %s: %s",
                 cfg.name.c_str(), e.what());
    return;
  }

  dm.has_header = extract_header(dm, dm.stamp, dm.frame_id);
  if (!dm.has_header) {
    dm.stamp = now();
    if (cfg.requires_tf)
      RCLCPP_WARN_ONCE(get_logger(), "topic %s requires tf but has no header",
                       cfg.name.c_str());
  }

  MessageEnvelope env;
  env.topic_name = cfg.name;
  env.type_string = cfg.type;
  env.data = std::move(dm);

  std::lock_guard<std::mutex> lock(sync_mutex_);
  auto &queue = message_queues_[topic_index];
  queue.push_back(std::move(env));

  if (queue.size() > queue_size_) {
    queue.pop_front();
    dropped_messages_count_++;
  }

  try_create_bundle();
}

void PerceptionNode::try_create_bundle() {
  // failing to find a match may leave queues in bundle-ready state
  // loop until at least one queue empty
  while (std::all_of(message_queues_.begin(), message_queues_.end(),
                     [](const auto &q) { return !q.empty(); })) {
    // choose anchor message as earliest message of all queues
    std::size_t anchor_queue_idx = 0;
    rclcpp::Time earliest_time = message_queues_[0].front().data.stamp;

    for (std::size_t i = 1; i < message_queues_.size(); ++i) {
      if (message_queues_[i].front().data.stamp < earliest_time) {
        earliest_time = message_queues_[i].front().data.stamp;
        anchor_queue_idx = i;
      }
    }

    rclcpp::Time anchor_time =
        message_queues_[anchor_queue_idx].front().data.stamp;
    std::vector<std::size_t> match_indices(message_queues_.size());

    // find closest message within slop_ for each queue
    // if any queue has no such message, abort attempt
    bool abort = false;
    for (std::size_t i = 0; i < message_queues_.size(); ++i) {
      if (i == anchor_queue_idx) {
        match_indices[i] = 0;
        continue;
      }

      bool found = false;
      double best_diff = std::numeric_limits<double>::max();
      for (std::size_t j = 0; j < message_queues_[i].size(); ++j) {
        double diff =
            (message_queues_[i][j].data.stamp - anchor_time).seconds();
        if (diff < -slop_)
          continue;
        if (std::abs(diff) <= slop_) {
          if (std::abs(diff) < best_diff) {
            best_diff = std::abs(diff);
            match_indices[i] = j;
            found = true;
          }
        } else if (diff > slop_)
          break;
      }

      if (!found) {
        message_queues_[anchor_queue_idx].pop_front();
        dropped_messages_count_++;
        abort = true;
        break;
      }
    }

    if (abort) {
      // failed anchor message has been popped, queues may now be
      // in bundle-ready state.
      // reattempt until at least one queue empty
      continue;
    }

    // bundle matched messages
    MessageBundle bundle;
    bundle.bundle_time = anchor_time;

    for (std::size_t i = 0; i < message_queues_.size(); ++i) {
      const auto &cfg = topic_configs_[i];
      auto env = std::move(message_queues_[i][match_indices[i]]);

      // drop messages from queues older than our matches
      for (std::size_t k = 0; k <= match_indices[i]; ++k) {
        if (!message_queues_[i].empty()) {
          if (k < match_indices[i])
            dropped_messages_count_++;
          message_queues_[i].pop_front();
        }
      }

      // resolve tf if required
      if (cfg.requires_tf && env.data.has_header) {
        auto tf = resolve_transform(cfg.target_frame, env.data.frame_id,
                                    env.data.stamp, cfg.max_tf_age);
        if (tf) {
          env.has_transform = true;
          env.transform = *tf;
        }
      }
      bundle.entries.emplace(env.topic_name, std::move(env));
    }

    publish_bundle_index(bundle);
    enqueue_bundle(std::move(bundle));
  }
}

std::optional<geometry_msgs::msg::TransformStamped>
PerceptionNode::resolve_transform(const std::string &target,
                                  const std::string &source,
                                  const rclcpp::Time &time, double max_age) {
  try {
    return tf_buffer_->lookupTransform(target, source, time);
  } catch (...) {
    try {
      auto latest =
          tf_buffer_->lookupTransform(target, source, tf2::TimePointZero);
      if (std::abs((rclcpp::Time(latest.header.stamp) - time).seconds()) <=
          max_age)
        return latest;
    } catch (...) {
    }
  }
  return std::nullopt;
}

void PerceptionNode::publish_bundle_index(const MessageBundle &bundle) {
  safepause_msgs::msg::MessageBundleIndex msg;
  msg.bundle_time = bundle.bundle_time;
  for (const auto &[_, entry] : bundle.entries) {
    safepause_msgs::msg::BundleIndexEntry e;
    e.topic_name = entry.topic_name;
    e.message_time = entry.data.stamp;
    e.has_transform = entry.has_transform;
    e.transform = entry.transform;
    msg.entries.push_back(std::move(e));
  }
  index_publisher_->publish(msg);
}

void PerceptionNode::enqueue_bundle(MessageBundle bundle) {
  std::lock_guard<std::mutex> lock(processing_mutex_);
  processing_queue_.push_back(std::move(bundle));
  processing_cv_.notify_one();
}

void PerceptionNode::processing_thread_main() {
  rclcpp::Rate rate(processing_rate_);
  while (running_ && rclcpp::ok()) {
    std::unique_lock<std::mutex> lock(processing_mutex_);
    processing_cv_.wait(
        lock, [this] { return !processing_queue_.empty() || !running_; });
    if (!running_)
      return;

    auto bundle = std::move(processing_queue_.front());
    processing_queue_.pop_front();
    lock.unlock();

    try {
      process_bundle(bundle);
    } catch (const std::exception &ex) {
      RCLCPP_ERROR(get_logger(), "%s", ex.what());
      rclcpp::shutdown();
    }

    size_t backlog;
    {
      std::lock_guard<std::mutex> inner_lock(processing_mutex_);
      backlog = processing_queue_.size();
    }
    if (backlog > 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "processing backlog: %zu items (total dropped: %lu)",
                           backlog, dropped_messages_count_.load());
    }
    rate.sleep();
  }
}

void PerceptionNode::process_bundle(const MessageBundle &bundle) {
  (void)bundle;

  // // if messages must be processed together,
  // // retrieve specific messages by topic name
  // const MessageEnvelope *point_cloud_env = nullptr;
  // const sensor_msgs::msg::PointCloud2 *point_cloud_msg = nullptr;
  // auto found = bundle.entries.find("/point_cloud");
  // if (found != bundle.entries.end()) {
  //   point_cloud_env = &found->second;
  //   point_cloud_msg =
  //   PerceptionNode::try_cast<sensor_msgs::msg::PointCloud2>(
  //       *point_cloud_env, "/point_cloud", "sensor_msgs/msg/PointCloud2",
  //       true);
  // }

  // const MessageEnvelope *pose_env = nullptr;
  // const geometry_msgs::msg::PoseStamped *pose_msg = nullptr;
  // found = bundle.entries.find("/pose");
  // if (found != bundle.entries.end()) {
  //   pose_env = &found->second;

  //   // downcast message to its original type
  //   // throw on unexpected type string and missing/unexpected transform
  //   pose_msg = PerceptionNode::try_cast<geometry_msgs::msg::PoseStamped>(
  //       *pose_env, "/pose", "geometry_msgs/msg/PoseStamped", true);
  // }

  // // process downcast messages together
  // if (point_cloud_msg && pose_msg) {
  //   // use messages and transforms
  //   (void)point_cloud_msg;
  //   (void)point_cloud_env->transform;

  //   (void)pose_msg;
  //   (void)pose_env->transform;
  // }

  // OR,

  // // if messages can be processed individually,
  // // iterate through bundle entries
  // for (const auto &[_, env] : bundle.entries) {

  //   if (env.type_string == "geometry_msgs/msg/PoseStamped") {
  //     auto *msg =
  //     PerceptionNode::try_cast<geometry_msgs::msg::PoseStamped>(
  //         env, env.topic_name, env.type_string, true);

  //     // use message and transform
  //     (void)msg;
  //     (void)env.transform;

  //   } else if (env.type_string == "sensor_msgs/msg/PointCloud2") {
  //     // ...
  //   }
  // }
}

std::string PerceptionNode::get_namespace_append(const std::string &ns) {
  return ns.empty() ? "" : "_" + ns;
}

} // namespace perception_pkg

int main(int argc, char **argv) {
  std::string ns_arg = "";
  std::vector<char *> remaining_argv;

  // parse for --namespace
  for (int i = 0; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--namespace" && i + 1 < argc) {
      ns_arg = argv[++i];
    } else {
      remaining_argv.push_back(argv[i]);
    }
  }

  int remaining_argc = static_cast<int>(remaining_argv.size());
  rclcpp::init(remaining_argc, remaining_argv.data());

  try {
    auto node = std::make_shared<perception_pkg::PerceptionNode>(ns_arg);
    rclcpp::spin(node);
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(
        rclcpp::get_logger(
            perception_pkg::NODE_NAME +
            perception_pkg::PerceptionNode::get_namespace_append(ns_arg)),
        "%s", ex.what());
  }

  rclcpp::shutdown();
  return 0;
}