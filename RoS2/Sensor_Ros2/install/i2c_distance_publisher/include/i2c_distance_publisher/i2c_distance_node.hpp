#ifndef I2C_DISTANCE_PUBLISHER__I2C_DISTANCE_NODE_HPP_
#define I2C_DISTANCE_PUBLISHER__I2C_DISTANCE_NODE_HPP_

#include <cstddef>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

namespace i2c_distance_publisher
{

class I2CDistanceNode : public rclcpp::Node
{
public:
  explicit I2CDistanceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~I2CDistanceNode() override;

private:
  void declare_parameters();
  void initialise_bus();
  void start_timer();
  void on_timer();

  std::optional<uint16_t> read_distance_raw();
  bool trigger_measurement();

  int read_register_byte(uint8_t reg);
  bool write_register_byte(uint8_t reg, uint8_t value);
  bool read_bytes(uint8_t reg, uint8_t * buffer, std::size_t length);

  std::string bus_path_;
  int fd_{-1};
  uint8_t device_address_{0x62};

  // Measurement parameters
  bool enable_trigger_{true};
  uint8_t trigger_register_{0x00};
  uint8_t trigger_value_{0x04};
  double trigger_delay_sec_{0.025};
  uint8_t result_register_high_{0x8f};
  uint8_t result_register_low_{0x10};
  bool contiguous_result_{false};
  double distance_scale_{0.01};
  double distance_offset_{0.0};

  // Publishing configuration
  std::string publish_topic_;
  std::string frame_id_;
  double poll_rate_hz_{10.0};
  double range_min_{0.05};
  double range_max_{2.0};
  double field_of_view_{0.052};

  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace i2c_distance_publisher

#endif  // I2C_DISTANCE_PUBLISHER__I2C_DISTANCE_NODE_HPP_
