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

#pragma once

#include <Eigen/Dense>
#include <cstdint>
#include <memory>

#include <boost/asio/io_context.hpp>

#include "serial_port.hpp"
#include "serial_port_options.hpp"

namespace witmotion_imu_driver_core
{
class WitmotionSerialImu
{
public:
  enum class CommunicationType : std::uint8_t
  {
    kStandard,
    kModbus,  /// @todo(Naoki Takahashi) not support
  };
  enum class DataType : std::uint8_t
  {
    kAcceleration = 1 << 0,
    kAngularVelocity = 1 << 1,
    kAngle = 1 << 2,
    kMagneticField = 1 << 3,
    kTemperature = 1 << 4,
    kVoltage = 1 << 5,
    kAll = 0xFF,
  };

  WitmotionSerialImu() = delete;
  explicit WitmotionSerialImu(
    boost::asio::io_context &, const SerialPortOptions &);
  ~WitmotionSerialImu() = default;

  bool connect();
  void disconnect();
  bool isConnected();

  float getGravityParam() const;
  void setGravityParam(float gravity);

  const Eigen::Vector3f & getAcceleleration();
  const Eigen::Vector3f & getAngularVelocity();
  const Eigen::Vector3f & getAngle();
  const Eigen::Vector3f & getMagneticField();
  float getTemperature() const;
  float getVoltage() const;

  void markAsRead(DataType);
  bool isSensorUpdated(DataType) const;
  bool hasSensorUpdated(DataType) const;

  void procSerialStream();

private:
  const SerialPortOptions options_;
  const CommunicationType communication_type_;

  float gravity_;

  std::unique_ptr<SerialPort> serial_port_;

  Eigen::Vector3f acceleration_;
  float temperature_{};
  Eigen::Vector3f angular_velocity_;
  float voltage_{};
  Eigen::Vector3f angle_;
  std::uint16_t version_{};
  Eigen::Vector3f magnetic_field_;

  DataType sensor_updated_;

  void loadSerialMsg(const SerialPort::Message &);
};

WitmotionSerialImu::DataType operator|(WitmotionSerialImu::DataType, WitmotionSerialImu::DataType);
WitmotionSerialImu::DataType operator&(WitmotionSerialImu::DataType, WitmotionSerialImu::DataType);
WitmotionSerialImu::DataType operator^(WitmotionSerialImu::DataType, WitmotionSerialImu::DataType);
WitmotionSerialImu::DataType operator!(WitmotionSerialImu::DataType);
}  // namespace witmotion_imu_driver_core
