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

  Eigen::Vector3f getAccel();

  void procSerialStream();
  void parseSerialMsg(const SerialPort::Message &);

private:
  const SerialPortOptions options_;
  const CommunicationType communication_type_;
  std::uint8_t device_id_;
  std::unique_ptr<SerialPort> serial_port_;
};
}  // namespace witmotion_imu_driver
