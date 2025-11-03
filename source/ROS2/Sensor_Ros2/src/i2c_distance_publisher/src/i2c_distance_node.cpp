#include "i2c_distance_publisher/i2c_distance_node.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <iomanip>
#include <sstream>
#include <optional>
#include <stdexcept>

namespace i2c_distance_publisher
{
namespace
{
constexpr double kDefaultPollRateHz = 10.0;
constexpr uint8_t kDefaultAddress = 0x62;
constexpr char kDefaultBusPath[] = "/dev/i2c-1";
constexpr double kDefaultDistanceScale = 0.01;  // centimetres -> metres
constexpr double kDefaultTriggerDelay = 0.025;  // 25 ms
}  // namespace

I2CDistanceNode::I2CDistanceNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("i2c_distance_node", options)
{
  declare_parameters();
  initialise_bus();

  publisher_ = create_publisher<sensor_msgs::msg::Range>(publish_topic_, rclcpp::QoS(10).reliable());

  start_timer();
  RCLCPP_INFO(get_logger(),
    "I2C distance publisher initialised on %s @ 0x%02x -> topic '%s'",
    bus_path_.c_str(), device_address_, publish_topic_.c_str());
}

I2CDistanceNode::~I2CDistanceNode()
{
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
}

void I2CDistanceNode::declare_parameters()
{
  bus_path_ = declare_parameter<std::string>("bus_path", kDefaultBusPath);
  device_address_ = static_cast<uint8_t>(declare_parameter<int>("device_address", kDefaultAddress));
  publish_topic_ = declare_parameter<std::string>("publish_topic", "/tb3_2/i2c_range_front");
  frame_id_ = declare_parameter<std::string>("frame_id", "tb3_2/i2c_front");
  poll_rate_hz_ = declare_parameter<double>("poll_rate_hz", kDefaultPollRateHz);
  range_min_ = declare_parameter<double>("range_min", 0.05);
  range_max_ = declare_parameter<double>("range_max", 2.0);
  field_of_view_ = declare_parameter<double>("field_of_view", 0.052);

  enable_trigger_ = declare_parameter<bool>("enable_trigger", true);
  trigger_register_ = static_cast<uint8_t>(declare_parameter<int>("trigger_register", 0x00));
  trigger_value_ = static_cast<uint8_t>(declare_parameter<int>("trigger_value", 0x04));
  trigger_delay_sec_ = declare_parameter<double>("trigger_delay_sec", kDefaultTriggerDelay);
  result_register_high_ = static_cast<uint8_t>(declare_parameter<int>("result_register_high", 0x8f));
  result_register_low_ = static_cast<uint8_t>(declare_parameter<int>("result_register_low", 0x10));
  contiguous_result_ = declare_parameter<bool>("contiguous_result", false);
  distance_scale_ = declare_parameter<double>("distance_scale", kDefaultDistanceScale);
  distance_offset_ = declare_parameter<double>("distance_offset", 0.0);
}

void I2CDistanceNode::initialise_bus()
{
  fd_ = open(bus_path_.c_str(), O_RDWR);
  if (fd_ < 0) {
    throw std::runtime_error("Failed to open " + bus_path_ + ": " + std::strerror(errno));
  }

  if (ioctl(fd_, I2C_SLAVE, device_address_) < 0) {
    const std::string err = std::strerror(errno);
    close(fd_);
    fd_ = -1;
    std::ostringstream ss;
    ss << std::hex << std::uppercase << std::setfill('0') << std::setw(2) << static_cast<int>(device_address_);
    throw std::runtime_error(
            "Failed to set I2C address 0x" + ss.str() + " on " + bus_path_ + ": " + err);
  }
}

void I2CDistanceNode::start_timer()
{
  if (poll_rate_hz_ <= 0.0) {
    poll_rate_hz_ = kDefaultPollRateHz;
  }
  const auto period = std::chrono::duration<double>(1.0 / poll_rate_hz_);
  timer_ = create_wall_timer(period, std::bind(&I2CDistanceNode::on_timer, this));
}

void I2CDistanceNode::on_timer()
{
  if (fd_ < 0) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "I2C bus not initialised.");
    return;
  }

  if (enable_trigger_) {
    if (!trigger_measurement()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Failed to trigger measurement.");
      return;
    }
    const auto delay = std::chrono::duration<double>(trigger_delay_sec_);
    if (delay.count() > 0.0) {
      const auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(delay);
      rclcpp::sleep_for(nanos);
    }
  }

  const auto raw = read_distance_raw();
  if (!raw.has_value()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to read distance from sensor.");
    return;
  }

  const double distance_m = static_cast<double>(raw.value()) * distance_scale_ + distance_offset_;
  if (distance_m <= 0.0 || distance_m > 100.0) {
    RCLCPP_DEBUG(get_logger(), "Ignoring distance %.3f m (raw %u)", distance_m, raw.value());
    return;
  }

  if (distance_m < range_min_ || distance_m > range_max_) {
    RCLCPP_DEBUG(get_logger(),
      "Distance %.3f m outside declared range [%.2f, %.2f], skipping publish",
      distance_m, range_min_, range_max_);
    return;
  }

  auto msg = sensor_msgs::msg::Range();
  msg.header.stamp = now();
  msg.header.frame_id = frame_id_;
  msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
  msg.field_of_view = static_cast<float>(field_of_view_);
  msg.min_range = static_cast<float>(range_min_);
  msg.max_range = static_cast<float>(range_max_);
  msg.range = static_cast<float>(distance_m);

  publisher_->publish(msg);
}

std::optional<uint16_t> I2CDistanceNode::read_distance_raw()
{
  if (contiguous_result_) {
    uint8_t buffer[2] = {0, 0};
    if (!read_bytes(result_register_high_, buffer, 2)) {
      return std::nullopt;
    }
    return static_cast<uint16_t>((buffer[0] << 8) | buffer[1]);
  }

  const int high = read_register_byte(result_register_high_);
  const int low = read_register_byte(result_register_low_);

  if (high < 0 || low < 0) {
    return std::nullopt;
  }
  return static_cast<uint16_t>((high << 8) | low);
}

bool I2CDistanceNode::trigger_measurement()
{
  return write_register_byte(trigger_register_, trigger_value_);
}

int I2CDistanceNode::read_register_byte(uint8_t reg)
{
  uint8_t value = 0;
  if (!read_bytes(reg, &value, 1)) {
    return -1;
  }
  return static_cast<int>(value);
}

bool I2CDistanceNode::write_register_byte(uint8_t reg, uint8_t value)
{
  uint8_t buffer[2] = {reg, value};
  struct i2c_msg message;
  std::memset(&message, 0, sizeof(message));
  message.addr = device_address_;
  message.flags = 0;
  message.len = sizeof(buffer);
  message.buf = buffer;

  struct i2c_rdwr_ioctl_data packets;
  std::memset(&packets, 0, sizeof(packets));
  packets.msgs = &message;
  packets.nmsgs = 1;

  if (ioctl(fd_, I2C_RDWR, &packets) < 0) {
    RCLCPP_DEBUG(get_logger(), "I2C write failed (reg 0x%02x): %s", reg, std::strerror(errno));
    return false;
  }
  return true;
}

bool I2CDistanceNode::read_bytes(uint8_t reg, uint8_t * buffer, std::size_t length)
{
  if (buffer == nullptr || length == 0) {
    return false;
  }

  uint8_t reg_buffer = reg;
  struct i2c_msg messages[2];
  std::memset(messages, 0, sizeof(messages));

  messages[0].addr = device_address_;
  messages[0].flags = 0;
  messages[0].len = 1;
  messages[0].buf = &reg_buffer;

  messages[1].addr = device_address_;
  messages[1].flags = I2C_M_RD;
  messages[1].len = static_cast<__u16>(length);
  messages[1].buf = buffer;

  struct i2c_rdwr_ioctl_data packets;
  std::memset(&packets, 0, sizeof(packets));
  packets.msgs = messages;
  packets.nmsgs = 2;

  if (ioctl(fd_, I2C_RDWR, &packets) < 0) {
    RCLCPP_DEBUG(get_logger(), "I2C read failed (reg 0x%02x len %zu): %s", reg, length, std::strerror(errno));
    return false;
  }
  return true;
}

}  // namespace i2c_distance_publisher

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<i2c_distance_publisher::I2CDistanceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
