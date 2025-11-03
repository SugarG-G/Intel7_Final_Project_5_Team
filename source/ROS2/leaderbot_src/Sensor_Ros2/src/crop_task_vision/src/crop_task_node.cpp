#include "crop_task_vision/crop_task_node.hpp"

#include <algorithm>
#include <array>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace crop_task_vision
{
namespace
{
constexpr char kDefaultCameraTopic[] = "/camera/image_raw";
constexpr char kDefaultStopTopic[] = "patrol/stop";
constexpr char kDefaultArmEventTopic[] = "crop_task/events";
constexpr char kDefaultArmDoneTopic[] = "crop_task/arm_done";
constexpr char kDefaultDebugTopic[] = "crop_task/debug_image";

int ensure_odd(int value)
{
  if (value <= 1) {
    return 1;
  }
  return (value % 2 == 0) ? value + 1 : value;
}
}  // namespace

CropTaskNode::CropTaskNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("crop_task_node", options)
{
  configure_parameters();
  initialise_interfaces();
  initialise_simulation_timer();

  RCLCPP_INFO(
    get_logger(),
    "Crop task node configured. Camera: %s, stop topic: %s, simulate arm: %s",
    camera_topic_.c_str(),
    stop_topic_.c_str(),
    simulate_arm_ ? "true" : "false");
}

void CropTaskNode::configure_parameters()
{
  camera_topic_ = declare_parameter<std::string>("camera_topic", kDefaultCameraTopic);
  stop_topic_ = declare_parameter<std::string>("stop_topic", kDefaultStopTopic);
  task_command_topic_ = declare_parameter<std::string>("task_command_topic", "");
  task_command_stop_ = declare_parameter<int>("task_command_stop", 2);
  task_command_resume_ = declare_parameter<int>("task_command_resume", 1);
  arm_event_topic_ = declare_parameter<std::string>("arm_event_topic", kDefaultArmEventTopic);
  arm_done_topic_ = declare_parameter<std::string>("arm_done_topic", kDefaultArmDoneTopic);
  debug_topic_ = declare_parameter<std::string>("debug_image_topic", kDefaultDebugTopic);

  min_area_ratio_ = declare_parameter<double>("min_area_ratio", min_area_ratio_);
  min_area_pixels_ = declare_parameter<double>("min_area_pixels", min_area_pixels_);
  blur_kernel_size_ = declare_parameter<int>("blur_kernel_size", blur_kernel_size_);
  morph_kernel_size_ = declare_parameter<int>("morph_kernel_size", morph_kernel_size_);
  morph_iterations_ = declare_parameter<int>("morph_iterations", morph_iterations_);
  detection_hold_frames_ = declare_parameter<int>("detection_hold_frames", detection_hold_frames_);
  release_frames_ = declare_parameter<int>("release_frames", release_frames_);
  simulate_arm_ = declare_parameter<bool>("simulate_arm", simulate_arm_);
  simulated_arm_duration_ = declare_parameter<double>("simulated_arm_duration", simulated_arm_duration_);
  publish_debug_image_ = declare_parameter<bool>("publish_debug_image", publish_debug_image_);

  const auto lower_param =
    declare_parameter<std::vector<double>>("hsv_lower", {hsv_lower_[0], hsv_lower_[1], hsv_lower_[2]});
  const auto upper_param =
    declare_parameter<std::vector<double>>("hsv_upper", {hsv_upper_[0], hsv_upper_[1], hsv_upper_[2]});

  if (lower_param.size() == 3) {
    hsv_lower_ = cv::Scalar(lower_param[0], lower_param[1], lower_param[2]);
  } else {
    RCLCPP_WARN(get_logger(), "Parameter hsv_lower must have exactly 3 elements. Using defaults.");
  }
  if (upper_param.size() == 3) {
    hsv_upper_ = cv::Scalar(upper_param[0], upper_param[1], upper_param[2]);
  } else {
    RCLCPP_WARN(get_logger(), "Parameter hsv_upper must have exactly 3 elements. Using defaults.");
  }

  if (blur_kernel_size_ < 0) {
    RCLCPP_WARN(get_logger(), "Negative blur kernel size provided. Resetting to 0.");
    blur_kernel_size_ = 0;
  }
  if (morph_kernel_size_ < 1) {
    morph_kernel_size_ = 1;
  }
  if (morph_iterations_ < 0) {
    morph_iterations_ = 0;
  }
  if (detection_hold_frames_ < 1) {
    detection_hold_frames_ = 1;
  }
  if (release_frames_ < detection_hold_frames_) {
    release_frames_ = detection_hold_frames_;
  }
  if (min_area_ratio_ < 0.0) {
    min_area_ratio_ = 0.0;
  }
  if (min_area_pixels_ < 0.0) {
    min_area_pixels_ = 0.0;
  }
  if (simulated_arm_duration_ < 0.0) {
    simulated_arm_duration_ = 0.0;
  }
}

void CropTaskNode::initialise_interfaces()
{
  using std::placeholders::_1;

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    camera_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&CropTaskNode::on_image, this, _1));

  stop_pub_ = create_publisher<std_msgs::msg::Bool>(stop_topic_, rclcpp::QoS(10).reliable());

  if (!task_command_topic_.empty() && task_command_stop_ >= 0 && task_command_resume_ >= 0) {
    task_command_pub_ =
      create_publisher<std_msgs::msg::UInt8>(task_command_topic_, rclcpp::QoS(10).best_effort());
  }

  if (!arm_event_topic_.empty()) {
    arm_event_pub_ =
      create_publisher<std_msgs::msg::String>(arm_event_topic_, rclcpp::QoS(10).reliable());
  }

  if (!arm_done_topic_.empty()) {
    arm_done_sub_ = create_subscription<std_msgs::msg::Bool>(
      arm_done_topic_, rclcpp::QoS(10).best_effort(),
      std::bind(&CropTaskNode::on_arm_done, this, _1));
  }

  if (publish_debug_image_) {
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>(
      debug_topic_, rclcpp::QoS(1).best_effort());
  }
}

void CropTaskNode::initialise_simulation_timer()
{
  if (!simulate_arm_) {
    return;
  }

  using namespace std::chrono_literals;
  sim_timer_ = create_wall_timer(
    100ms, std::bind(&CropTaskNode::on_sim_timer, this));
}

void CropTaskNode::on_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & ex) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "cv_bridge conversion failed: %s", ex.what());
    return;
  }

  cv::Mat mask;
  cv::Rect roi;
  const bool detected = detect_object(cv_ptr->image, mask, roi);
  rclcpp::Time stamp;
  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
    stamp = get_clock()->now();
  } else {
    stamp = rclcpp::Time(msg->header.stamp);
  }

  process_detection(stamp, detected, roi);

  publish_debug(*msg, cv_ptr->image, mask, roi, detected);
}

bool CropTaskNode::detect_object(
  const cv::Mat & frame,
  cv::Mat & mask,
  cv::Rect & best_roi) const
{
  if (frame.empty()) {
    return false;
  }

  cv::Mat hsv;
  cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
  cv::inRange(hsv, hsv_lower_, hsv_upper_, mask);

  if (blur_kernel_size_ > 1) {
    const int kernel = ensure_odd(blur_kernel_size_);
    cv::medianBlur(mask, mask, kernel);
  }

  if (morph_iterations_ > 0) {
    const int kernel_size = ensure_odd(morph_kernel_size_);
    cv::Mat element = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
    cv::morphologyEx(
      mask, mask, cv::MORPH_CLOSE, element, cv::Point(-1, -1), morph_iterations_);
  }

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  const double frame_area = static_cast<double>(frame.cols * frame.rows);
  const double threshold_area = std::max(min_area_pixels_, min_area_ratio_ * frame_area);

  double best_area = 0.0;
  cv::Rect candidate;
  for (const auto & contour : contours) {
    const double area = cv::contourArea(contour);
    if (area < threshold_area) {
      continue;
    }
    const cv::Rect rect = cv::boundingRect(contour);
    if (area > best_area) {
      best_area = area;
      candidate = rect;
    }
  }

  if (best_area > 0.0) {
    best_roi = candidate;
    return true;
  }

  best_roi = cv::Rect();
  return false;
}

void CropTaskNode::process_detection(
  const rclcpp::Time & stamp,
  bool detected,
  const cv::Rect & roi)
{
  if (detected) {
    detection_hits_ = std::min(detection_hits_ + 1, detection_hold_frames_);
    detection_misses_ = 0;
  } else {
    detection_misses_ = std::min(detection_misses_ + 1, release_frames_);
    detection_hits_ = 0;
  }

  if (state_ == TaskState::COOLDOWN && detection_misses_ >= release_frames_) {
    state_ = TaskState::RUNNING;
    detection_latched_ = false;
    detection_hits_ = 0;
    detection_misses_ = 0;
    RCLCPP_INFO(get_logger(), "Cooldown complete. Detector re-armed.");
  }

  if (!detection_latched_ && state_ == TaskState::RUNNING && detected &&
    detection_hits_ >= detection_hold_frames_)
  {
    detection_latched_ = true;
    last_detection_stamp_ = stamp;
    trigger_stop(stamp, roi);
  }
}

void CropTaskNode::trigger_stop(const rclcpp::Time & stamp, const cv::Rect & roi)
{
  if (state_ != TaskState::RUNNING) {
    return;
  }

  state_ = TaskState::STOPPING;

  std_msgs::msg::Bool stop_msg;
  stop_msg.data = true;
  stop_pub_->publish(stop_msg);

  if (task_command_pub_) {
    std_msgs::msg::UInt8 cmd;
    cmd.data = static_cast<uint8_t>(task_command_stop_);
    task_command_pub_->publish(cmd);
  }

  if (arm_event_pub_) {
    std_msgs::msg::String evt;
    evt.data = "crop_detected";
    arm_event_pub_->publish(evt);
  }

  RCLCPP_WARN(
    get_logger(),
    "Crop surrogate detected at t=%.3f. Bounding box x=%d y=%d w=%d h=%d",
    stamp.seconds(),
    roi.x, roi.y, roi.width, roi.height);

  start_arm_sequence(stamp);
}

void CropTaskNode::start_arm_sequence(const rclcpp::Time & stamp)
{
  state_ = TaskState::ARM_ACTIVE;
  arm_sequence_active_ = true;

  if (arm_event_pub_) {
    std_msgs::msg::String evt;
    evt.data = "arm_pick_start";
    arm_event_pub_->publish(evt);
  }

  if (simulate_arm_) {
    sim_deadline_ = stamp + rclcpp::Duration::from_seconds(simulated_arm_duration_);
  }
}

void CropTaskNode::complete_arm_sequence(const rclcpp::Time & stamp)
{
  if (!arm_sequence_active_) {
    return;
  }

  arm_sequence_active_ = false;

  if (arm_event_pub_) {
    std_msgs::msg::String evt;
    evt.data = "arm_pick_complete";
    arm_event_pub_->publish(evt);
  }

  trigger_resume(stamp);
}

void CropTaskNode::trigger_resume(const rclcpp::Time & stamp)
{
  std_msgs::msg::Bool resume_msg;
  resume_msg.data = false;
  stop_pub_->publish(resume_msg);

  if (task_command_pub_) {
    std_msgs::msg::UInt8 cmd;
    cmd.data = static_cast<uint8_t>(task_command_resume_);
    task_command_pub_->publish(cmd);
  }

  last_resume_stamp_ = stamp;
  state_ = TaskState::COOLDOWN;
  detection_misses_ = 0;

  if (arm_event_pub_) {
    std_msgs::msg::String evt;
    evt.data = "patrol_resume";
    arm_event_pub_->publish(evt);
  }

  RCLCPP_INFO(
    get_logger(),
    "Resume issued at t=%.3f. Waiting for cooldown (%d frames without detection).",
    stamp.seconds(),
    release_frames_);
}

void CropTaskNode::publish_debug(
  const sensor_msgs::msg::Image & input_msg,
  const cv::Mat & frame,
  const cv::Mat & mask,
  const cv::Rect & roi,
  bool detected)
{
  if (!publish_debug_image_ || !debug_pub_) {
    return;
  }

  cv::Mat annotated;
  frame.copyTo(annotated);
  if (detected && roi.area() > 0) {
    cv::rectangle(annotated, roi, cv::Scalar(0, 255, 0), 2);
  }

  cv::Mat mask_color;
  if (!mask.empty()) {
    cv::cvtColor(mask, mask_color, cv::COLOR_GRAY2BGR);
  } else {
    mask_color = cv::Mat::zeros(annotated.size(), CV_8UC3);
  }

  cv::Mat combined;
  cv::hconcat(annotated, mask_color, combined);

  cv_bridge::CvImage debug_msg;
  debug_msg.header = input_msg.header;
  debug_msg.encoding = sensor_msgs::image_encodings::BGR8;
  debug_msg.image = combined;

  debug_pub_->publish(*debug_msg.toImageMsg());
}

void CropTaskNode::on_arm_done(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data) {
    return;
  }

  const rclcpp::Time now = get_clock()->now();
  RCLCPP_INFO(get_logger(), "Received external arm completion signal.");
  complete_arm_sequence(now);
}

void CropTaskNode::on_sim_timer()
{
  if (!simulate_arm_ || !arm_sequence_active_) {
    return;
  }

  const rclcpp::Time now = get_clock()->now();
  if (now >= sim_deadline_) {
    RCLCPP_INFO(get_logger(), "Simulated arm task finished after %.2f seconds.", simulated_arm_duration_);
    complete_arm_sequence(now);
  }
}

}  // namespace crop_task_vision
