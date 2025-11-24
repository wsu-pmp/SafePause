#include <rclcpp/parameter_value.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "safepause_msgs/srv/start_recording.hpp"
#include "training_recorder/recorder_node.hpp"
#include "training_recorder/topic_handler.hpp"

class RecordingController : public rclcpp::Node {
public:
  RecordingController(std::shared_ptr<training_recorder::RecorderNode> recorder)
      : rclcpp::Node("recording_controller"), recorder_(recorder) {
    // defaults applied by recorder_node
    this->declare_parameter("num_callback_threads", -1);
    this->declare_parameter("num_writer_threads", -1);
    this->declare_parameter("queue_capacity", -1);
    this->declare_parameter("buffer_size_bytes", -1);
    this->declare_parameter("pool_size", -1);

    start_srv_ = this->create_service<safepause_msgs::srv::StartRecording>(
        "recording_controller/start",
        std::bind(&RecordingController::startCB, this, std::placeholders::_1,
                  std::placeholders::_2));

    close_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "recording_controller/stop",
        std::bind(&RecordingController::stopCB, this, std::placeholders::_1,
                  std::placeholders::_2));
  }

private:
  void startCB(
      const std::shared_ptr<safepause_msgs::srv::StartRecording::Request> req,
      std::shared_ptr<safepause_msgs::srv::StartRecording::Response> res) {
    const auto &[success, message] = this->recorder_->start(req->storage_path);
    res->success = success;
    res->message = message;
  }

  void stopCB(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
              std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    const auto &[success, message] = this->recorder_->stop();
    res->success = success;
    res->message = message;

    this->recorder_->get_profiler_snapshot();
  }

  std::shared_ptr<training_recorder::RecorderNode> recorder_;

  rclcpp::Service<safepause_msgs::srv::StartRecording>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr close_srv_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // topics
  std::vector<std::unique_ptr<training_recorder::TopicHandlerFactoryBase>>
      topic_factories;
  topic_factories.push_back(
      std::make_unique<
          training_recorder::TopicHandlerFactory<sensor_msgs::msg::JointState>>(
          "joint_states"));
  topic_factories.push_back(
      std::make_unique<
          training_recorder::TopicHandlerFactory<std_msgs::msg::String>>(
          "chatter"));

  // RecorderNode handles its own executor
  auto recorder = std::make_shared<training_recorder::RecorderNode>(
      std::move(topic_factories));

  // ControllerNode exposes services and calls RecorderNode functions
  auto controller = std::make_shared<RecordingController>(recorder);

  if (recorder->init()) {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(controller);

    RCLCPP_INFO(controller->get_logger(), "Services:");
    RCLCPP_INFO(controller->get_logger(), "\trecording_controller/start");
    RCLCPP_INFO(controller->get_logger(),
                "\t\tsafepause_mgs/srv/StartRecording");
    RCLCPP_INFO(controller->get_logger(), "\trecording_controller/stop");
    RCLCPP_INFO(controller->get_logger(), "\t\tstd_srvs/srv/Trigger");

    exec.spin();
  }

  rclcpp::shutdown();

  return 0;
}