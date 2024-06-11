#pragma once

#include <cstdint>
#include <memory>

#include <Eigen/Dense>
#include <boost/asio//io_context.hpp>

#include "serial_port_options.hpp"
#include "serial_port.hpp"

namespace witmotion_imu_driver
{
class WitmotionSerialImu
{
public:
  enum class CommunicationType : std::uint8_t
  {
    STANDARD,
    MODBUS,  /// @todo(Naoki Takahashi) not support
  };

  WitmotionSerialImu() = delete;
  explicit WitmotionSerialImu(
    boost::asio::io_context &, const SerialPortOptions &, const std::uint8_t device_id);
  ~WitmotionSerialImu() = default;

  bool connect();
  bool isConnected();
  void changeDeviceId(const std::uint8_t device_id);
  std::uint8_t getDeviceId() const;

  bool searchDevice();
  bool searchDevice(const std::uint8_t overrite_device_id);

  const Eigen::Vector3f & getAcceleleration();
  const Eigen::Vector3f & getAngularVelocity();
  const Eigen::Vector3f & getAngle();
  const Eigen::Vector3f & getMagneticField();
  float getTemperature();
  float getVoltage();

  bool isUpdatedAcceleration() const;
  bool isUpdatedAngularVelocity() const;
  bool isUpdatedAngle() const;
  bool isUpdatedMagneticField() const;

  void procSerialStream();

private:
  const SerialPortOptions options_;
  const CommunicationType communication_type_;

  std::uint8_t device_id_;
  float gravity_;

  std::unique_ptr<SerialPort> serial_port_;

  Eigen::Vector3f acceleration_;
  float temperature_;
  Eigen::Vector3f angular_velocity_;
  float voltage_;
  Eigen::Vector3f angle_;
  std::uint16_t version_;
  Eigen::Vector3f magnetic_field_;

  bool acceleration_updated_;
  bool angular_velocity_updated_;
  bool angle_updated_;
  bool magnetic_field_updated_;

  void loadSerialMsg(const SerialPort::Message &);
};
}  // namespace witmotion_imu_driver
