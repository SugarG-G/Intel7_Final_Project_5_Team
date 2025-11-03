#ifndef CROP_TASK_VISION__CROP_TASK_NODE_HPP_
#define CROP_TASK_VISION__CROP_TASK_NODE_HPP_

#include <string>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace crop_task_vision
{

class CropTaskNode : public rclcpp::Node
{
public:
  explicit CropTaskNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  enum class TaskState
  {
    RUNNING,
    STOPPING,
    ARM_ACTIVE,
    COOLDOWN
  };

  void configure_parameters();
  void initialise_interfaces();
  void initialise_simulation_timer();

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void on_arm_done(const std_msgs::msg::Bool::SharedPtr msg);
  void on_sim_timer();

  bool detect_object(
    const cv::Mat & frame,
    cv::Mat & mask,
    cv::Rect & best_roi) const;

  void process_detection(
    const rclcpp::Time & stamp,
    bool detected,
    const cv::Rect & roi);

  void trigger_stop(const rclcpp::Time & stamp, const cv::Rect & roi);
  void trigger_resume(const rclcpp::Time & stamp);
  void start_arm_sequence(const rclcpp::Time & stamp);
  void complete_arm_sequence(const rclcpp::Time & stamp);
  void publish_debug(
    const sensor_msgs::msg::Image & input_msg,
    const cv::Mat & frame,
    const cv::Mat & mask,
    const cv::Rect & roi,
    bool detected);

  // Parameters
  std::string camera_topic_;
  std::string stop_topic_;
  std::string task_command_topic_;
  std::string arm_event_topic_;
  std::string arm_done_topic_;
  std::string debug_topic_;
  int task_command_stop_{-1};
  int task_command_resume_{-1};
  double min_area_ratio_{0.01};
  double min_area_pixels_{1500.0};
  int blur_kernel_size_{5};
  int morph_kernel_size_{5};
  int morph_iterations_{2};
  int detection_hold_frames_{4};
  int release_frames_{8};
  bool simulate_arm_{true};
  double simulated_arm_duration_{5.0};
  bool publish_debug_image_{false};

  cv::Scalar hsv_lower_{20.0, 100.0, 100.0};
  cv::Scalar hsv_upper_{40.0, 255.0, 255.0};

  // Runtime state
  TaskState state_{TaskState::RUNNING};
  bool detection_latched_{false};
  int detection_hits_{0};
  int detection_misses_{0};
  bool arm_sequence_active_{false};
  rclcpp::Time last_detection_stamp_;
  rclcpp::Time last_resume_stamp_;
  rclcpp::Time sim_deadline_;

  // Interfaces
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_done_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr task_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arm_event_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr sim_timer_;
};

}  // namespace crop_task_vision

#endif  // CROP_TASK_VISION__CROP_TASK_NODE_HPP_
