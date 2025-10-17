#pragma once

#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <rclcpp/node.hpp>
#include <set>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace event_logger {

constexpr int SCHEMA = 1;

inline constexpr const char *PARAM_NAME = "event_log_path";
inline constexpr const char *ENV_NAME = "EVENT_LOG_PATH";
inline constexpr const char *ENV_ROS_HOME = "ROS_HOME";
inline constexpr const char *DEFAULT_SUBDIR = "event_logs";

/// Append-only jsonl event log, one file per logger per process.
///
/// The output directory comes from the node's `event_log_path` parameter, or
/// the EVENT_LOG_PATH environment variable if that parameter is unset.
class EventLogger {
public:
  EventLogger(rclcpp::Node *node, const std::string &name,
              const nlohmann::json &meta = nlohmann::json::object())
      : node_(node), name_(name), pid_(static_cast<int>(::getpid())) {
    node_name_ = node_->get_fully_qualified_name();
    host_ = host_name();

    const std::string base_path = resolve_base_path(node_);

    // same name twice in one process would mean two handles appending to one
    // file with independent seq counters
    {
      std::lock_guard<std::mutex> lock(open_names_mutex());
      if (!open_names().insert(name_).second) {
        throw std::runtime_error("EventLogger '" + name_ +
                                 "' already open in this process (pid " +
                                 std::to_string(pid_) + ")");
      }
      registered_ = true;
    }

    const std::time_t started = std::time(nullptr);
    std::tm tm_started{};
    localtime_r(&started, &tm_started);

    try {
      std::filesystem::path dir =
          std::filesystem::path(base_path) / fmt_time("%Y-%m-%d", tm_started);
      std::filesystem::create_directories(dir);

      // pid discriminates replicas launched from one launch file, which would
      // otherwise collide whenever two start within the same second
      path_ = dir / (name_ + "_" + fmt_time("%Y%m%d_%H%M%S", tm_started) + "_" +
                     std::to_string(pid_) + ".jsonl");

      if (std::filesystem::exists(path_)) {
        throw std::runtime_error("event log already exists: " + path_.string());
      }

      file_.open(path_, std::ios::out);
      if (!file_) {
        throw std::runtime_error("cannot open event log: " + path_.string());
      }
    } catch (...) {
      release_name();
      throw;
    }

    emit("open", meta);
  }

  ~EventLogger() { close(); }

  EventLogger(const EventLogger &) = delete;
  EventLogger &operator=(const EventLogger &) = delete;

  void log(const nlohmann::json &event) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_)
      return;
    try {
      emit("event", event);
    } catch (const std::exception &e) {
      ++write_errors_;
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                            "event log write failed (%d so far): %s",
                            write_errors_, e.what());
    }
  }

  /// write closing record and release the file
  void close() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_)
      return;
    closed_ = true;
    try {
      emit("close", nlohmann::json{{"write_errors", write_errors_}});
    } catch (...) {
    }
    file_.close();
    release_name();
  }

  const std::filesystem::path &path() const { return path_; }

private:
  static std::set<std::string> &open_names() {
    static std::set<std::string> names;
    return names;
  }

  static std::mutex &open_names_mutex() {
    static std::mutex m;
    return m;
  }

  static std::string fmt_time(const char *format, const std::tm &tm) {
    char buf[32];
    std::strftime(buf, sizeof(buf), format, &tm);
    return std::string(buf);
  }

  static std::string host_name() {
    char buf[256];
    if (gethostname(buf, sizeof(buf)) != 0)
      return "";
    buf[sizeof(buf) - 1] = '\0';
    return std::string(buf);
  }

  static std::string resolve_base_path(rclcpp::Node *node) {
    // declared here so the parameter works without every consumer having to
    // declare it first
    if (!node->has_parameter(PARAM_NAME)) {
      node->declare_parameter(PARAM_NAME, "");
    }

    std::string base_path = node->get_parameter(PARAM_NAME).as_string();
    if (base_path.empty()) {
      if (const char *env = std::getenv(ENV_NAME))
        base_path = env;
    }
    if (base_path.empty()) {
      // defaults to ROS_HOME
      std::filesystem::path ros_home;
      if (const char *env = std::getenv(ENV_ROS_HOME)) {
        ros_home = env;
      } else if (const char *home = std::getenv("HOME")) {
        ros_home = std::filesystem::path(home) / ".ros";
      } else {
        ros_home = std::filesystem::temp_directory_path();
      }
      base_path = (ros_home / DEFAULT_SUBDIR).string();
    }
    return base_path;
  }

  void release_name() {
    if (!registered_)
      return;
    std::lock_guard<std::mutex> lock(open_names_mutex());
    open_names().erase(name_);
    registered_ = false;
  }

  void emit(const char *record_type, const nlohmann::json &data) {
    const auto stamp = node_->get_clock()->now();
    const auto wall = std::chrono::duration<double>(
                          std::chrono::system_clock::now().time_since_epoch())
                          .count();

    // key order matches the Python implementation's insertion order so the two
    // produce identical lines
    nlohmann::ordered_json record;
    record["schema"] = SCHEMA;
    record["type"] = record_type;
    record["logger"] = name_;
    record["node"] = node_name_;
    record["pid"] = pid_;
    record["host"] = host_;
    record["seq"] = seq_++;
    // include both ROS timestamp and wall time
    record["stamp"] = {{"sec", stamp.nanoseconds() / 1000000000},
                       {"nanosec", stamp.nanoseconds() % 1000000000}};
    record["wall"] = wall;
    record["data"] = data;

    file_ << record.dump() << "\n";
    file_.flush();
    if (!file_) {
      throw std::runtime_error("write failed");
    }
  }

  rclcpp::Node *node_;
  std::string name_;
  std::string node_name_;
  std::string host_;
  int pid_;

  std::filesystem::path path_;
  std::ofstream file_;
  std::mutex mutex_;
  std::uint64_t seq_{0};
  int write_errors_{0};
  bool closed_{false};
  bool registered_{false};
};

} // namespace event_logger
