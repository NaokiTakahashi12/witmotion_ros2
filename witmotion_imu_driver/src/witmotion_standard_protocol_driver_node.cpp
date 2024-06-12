// MIT License
//
// Copyright (c) 2024 Naoki Takahashi
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <Eigen/Dense>
#include <chrono>
#include <functional>
#include <memory>

#include <boost/asio/io_context.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/header.hpp>
#include <witmotion_standard_protocol_driver_node_params.hpp>

#include "serial_port_options.hpp"
#include "witmotion_serial_imu.hpp"

namespace witmotion_imu_driver
{
class WitmotionStandardProtocolDriverNode : public rclcpp::Node
{
public:
  WitmotionStandardProtocolDriverNode() = delete;
  explicit WitmotionStandardProtocolDriverNode(const rclcpp::NodeOptions &);
  ~WitmotionStandardProtocolDriverNode() override;

private:
  boost::asio::io_context io_context_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_publisher_;

  rclcpp::TimerBase::SharedPtr proc_imu_state_timer_;
  rclcpp::TimerBase::SharedPtr publish_imu_state_timer_;

  std::unique_ptr<witmotion_standard_protocol_driver_node::ParamListener> param_listener_;
  std::unique_ptr<witmotion_standard_protocol_driver_node::Params> params_;

  std::unique_ptr<WitmotionSerialImu> witmotion_serial_imu_;

  void procSerialImuTimerCallback();
  void publishImuStateTimerCallback();
};

WitmotionStandardProtocolDriverNode::WitmotionStandardProtocolDriverNode(
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node("witmotion_standard_protocol_driver", node_options),

  imu_publisher_(nullptr),
  proc_imu_state_timer_(nullptr),
  publish_imu_state_timer_(nullptr),
  param_listener_(nullptr),
  params_(nullptr),
  witmotion_serial_imu_(nullptr)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Start " << this->get_name());

  param_listener_ = std::make_unique<witmotion_standard_protocol_driver_node::ParamListener>(
    this->get_node_parameters_interface());
  params_ = std::make_unique<witmotion_standard_protocol_driver_node::Params>(
    param_listener_->get_params());

  imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("~/imu/raw", 3);
  temperature_publisher_ =
    this->create_publisher<sensor_msgs::msg::Temperature>("~/temperature", 3);
  voltage_publisher_ = this->create_publisher<std_msgs::msg::Float32>("~/voltage", 3);

  SerialPortOptions serial_port_options;
  serial_port_options.device_port_name = params_->device_port_name;
  serial_port_options.baud_rate = params_->serial_baud_rate;
  witmotion_serial_imu_ = std::make_unique<WitmotionSerialImu>(
    io_context_, serial_port_options, params_->serial_device_id);

  if (witmotion_serial_imu_->isConnected()) {
    RCLCPP_INFO_STREAM(
      this->get_logger(), "Connect serial device " << params_->device_port_name << " successful");
  }

  const auto proc_imu_duration_milliseconds =
    static_cast<unsigned int>(1e3 / params_->proc_serial_frequency);
  proc_imu_state_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(proc_imu_duration_milliseconds),
    std::bind(&WitmotionStandardProtocolDriverNode::procSerialImuTimerCallback, this));

  const auto publish_imu_duration_milliseconds =
    static_cast<unsigned int>(1e3 / params_->publish_frequency);
  publish_imu_state_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(publish_imu_duration_milliseconds),
    std::bind(&WitmotionStandardProtocolDriverNode::publishImuStateTimerCallback, this));
}

WitmotionStandardProtocolDriverNode::~WitmotionStandardProtocolDriverNode()
{
  if (witmotion_serial_imu_) {
    witmotion_serial_imu_.reset();
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Finish " << this->get_name());
}

void WitmotionStandardProtocolDriverNode::procSerialImuTimerCallback()
{
  witmotion_serial_imu_->procSerialStream();
}

Eigen::Quaternionf quaternionFromEularAngles(const Eigen::Vector3f & a)
{
  Eigen::Quaternionf q;
  q = Eigen::AngleAxisf(a.z(), Eigen::Vector3f::UnitZ());
  q = q * Eigen::AngleAxisf(a.y(), Eigen::Vector3f::UnitY());
  q = q * Eigen::AngleAxisf(a.x(), Eigen::Vector3f::UnitX());
  return q;
}

void WitmotionStandardProtocolDriverNode::publishImuStateTimerCallback()
{
  const bool updated_acceleration = witmotion_serial_imu_->isUpdatedAcceleration();
  const bool updated_angular_velocity = witmotion_serial_imu_->isUpdatedAngularVelocity();
  const bool updated_angle = witmotion_serial_imu_->isUpdatedAngle();
  const bool allow_publish_imu_topic =
    updated_acceleration && updated_angular_velocity && updated_angle;

  std_msgs::msg::Header msg_header;
  msg_header.frame_id = params_->frame_id;
  msg_header.stamp = this->get_clock()->now();

  if (imu_publisher_ && allow_publish_imu_topic) {
    auto msg = std::make_unique<sensor_msgs::msg::Imu>();
    const auto acceleration = witmotion_serial_imu_->getAcceleleration();
    const auto angular_velocity = witmotion_serial_imu_->getAngularVelocity();
    msg->header = msg_header;
    if (params_->angle_to_quaternion) {
      const auto quaternion = quaternionFromEularAngles(witmotion_serial_imu_->getAngle());
      msg->orientation.x = quaternion.x();
      msg->orientation.y = quaternion.y();
      msg->orientation.z = quaternion.z();
      msg->orientation.w = quaternion.w();
    }
    msg->linear_acceleration.x = acceleration.x();
    msg->linear_acceleration.y = acceleration.y();
    msg->linear_acceleration.z = acceleration.z();
    msg->angular_velocity.x = angular_velocity.x();
    msg->angular_velocity.y = angular_velocity.y();
    msg->angular_velocity.z = angular_velocity.z();
    imu_publisher_->publish(std::move(msg));
  }
  if (temperature_publisher_ && updated_acceleration) {
    auto msg = std::make_unique<sensor_msgs::msg::Temperature>();
    msg->header = msg_header;
    msg->temperature = witmotion_serial_imu_->getTemperature();
    msg->variance = 0.0;
    temperature_publisher_->publish(std::move(msg));
  }
  if (voltage_publisher_ && updated_angular_velocity) {
    auto msg = std::make_unique<std_msgs::msg::Float32>();
    msg->data = witmotion_serial_imu_->getVoltage();
    voltage_publisher_->publish(std::move(msg));
  }
}
}  // namespace witmotion_imu_driver

RCLCPP_COMPONENTS_REGISTER_NODE(witmotion_imu_driver::WitmotionStandardProtocolDriverNode)
