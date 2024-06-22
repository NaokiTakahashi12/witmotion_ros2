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

#include <witmotion_imu_driver_core/witmotion_serial_imu.hpp>

#include <memory>
#include <numeric>

#include <witmotion_imu_driver_core/serial_port.hpp>
#include <witmotion_imu_driver_core/serial_port_options.hpp>
#include <witmotion_imu_driver_core/standard_protocol.hpp>

namespace witmotion_imu_driver_core
{
WitmotionSerialImu::WitmotionSerialImu(
  boost::asio::io_context & io_context,
  const SerialPortOptions & options)
: options_(options),
  communication_type_(CommunicationType::kStandard),
  gravity_(9.82F),
  serial_port_(std::make_unique<SerialPort>(io_context, options))
{
  if (connect()) {
    throw std::runtime_error("Failed connect serial port");
  }
}

bool WitmotionSerialImu::connect()
{
  return serial_port_->open();
}

void WitmotionSerialImu::disconnect()
{
  serial_port_->close();
}

bool WitmotionSerialImu::isConnected()
{
  return serial_port_->isOpen();
}

float WitmotionSerialImu::getGravityParam() const
{
  return gravity_;
}

void WitmotionSerialImu::setGravityParam(float gravity)
{
  gravity_ = gravity;
}

namespace standard
{
inline float convertDoubleByteToFloat(const std::uint8_t a, const std::uint8_t b)
{
  const std::uint16_t c = (a << 8) | b;
  std::int16_t ret = 0;
  std::memcpy(&ret, &c, sizeof(std::int16_t));
  return static_cast<float>(ret);
}

inline Eigen::Vector3f parseVector3(const SerialPort::Message & msg)
{
  constexpr unsigned int kVectorDimention = 3;
  Eigen::Vector3f vec;
  for (unsigned int i = 0; i < kVectorDimention; ++i) {
    const unsigned int head = 2 * i + std_packet_size::kMsgHeader;
    vec(i) = convertDoubleByteToFloat(msg[head + 1], msg[head]);
  }
  return vec;
}

inline void parseAcceleration(
  Eigen::Vector3f & acceleration, float & templerature, const SerialPort::Message & msg)
{
  constexpr float kAccelerationScaler = 1.0F / 32758.0F * 16.0F;
  constexpr float kTemperatureScaler = 1.0F / 100.0F;
  acceleration = kAccelerationScaler * parseVector3(msg);
  templerature = kTemperatureScaler * convertDoubleByteToFloat(msg[9], msg[8]);
}

inline void parseAngularVelocity(
  Eigen::Vector3f & angular_velocity, float & voltage, const SerialPort::Message & msg)
{
  constexpr float kAngularVelocityScaler = 1.0F / 32758.0F * 2000.0F * M_PI / 180.0F;
  constexpr float kVoltageScaler = 1.0F / 100.0F;
  angular_velocity = kAngularVelocityScaler * parseVector3(msg);
  voltage = kVoltageScaler * convertDoubleByteToFloat(msg[9], msg[8]);
}

inline void parseAngle(
  Eigen::Vector3f & angle, std::uint16_t & version, const SerialPort::Message & msg)
{
  constexpr float kAngleScaler = 1.0F / 32758.0F * 180.0F * M_PI / 180.0F;
  angle = kAngleScaler * parseVector3(msg);
  version = convertDoubleByteToFloat(msg[9], msg[8]);
}

inline void parseMagneticField(Eigen::Vector3f & magnetic_field, const SerialPort::Message & msg)
{
  magnetic_field = parseVector3(msg);
}
}  // namespace standard

const Eigen::Vector3f & WitmotionSerialImu::getAcceleleration()
{
  return acceleration_;
}

const Eigen::Vector3f & WitmotionSerialImu::getAngularVelocity()
{
  return angular_velocity_;
}

const Eigen::Vector3f & WitmotionSerialImu::getAngle()
{
  return angle_;
}

const Eigen::Vector3f & WitmotionSerialImu::getMagneticField()
{
  return magnetic_field_;
}

float WitmotionSerialImu::getTemperature() const
{
  return temperature_;
}

float WitmotionSerialImu::getVoltage() const
{
  return voltage_;
}

void WitmotionSerialImu::markAsRead(DataType data_type)
{
  sensor_updated_ = sensor_updated_ & (DataType::kAll ^ data_type);
}

bool WitmotionSerialImu::isSensorUpdated(DataType data_type) const
{
  return (sensor_updated_ & data_type) == data_type;
}

bool WitmotionSerialImu::hasSensorUpdated(DataType data_type) const
{
  return (sensor_updated_ & data_type) > DataType(0);
}

void WitmotionSerialImu::procSerialStream()
{
  if (communication_type_ == CommunicationType::kStandard) {
    constexpr std::size_t kProcMsgs = 4;
    static SerialPort::Message header_msg{std_packet_type::kStdHeader};
    for (unsigned int i = 0; i < kProcMsgs; ++i) {
      loadSerialMsg(serial_port_->read(std_packet_size::kStreamMsg, header_msg));
    }
  } else if (communication_type_ == CommunicationType::kModbus) {
    throw std::logic_error("Not supported Modbus");
  }
}

void WitmotionSerialImu::loadSerialMsg(const SerialPort::Message & msg)
{
  const std::uint8_t crc = std::accumulate(msg.cbegin(), msg.cend() - 1, 0U);
  if (crc != msg[std_packet_index::kMsgCrc]) {
    return;
  }
  switch (msg[std_packet_index::kMsgType]) {
    case std_packet_type::kAcceleration:
      standard::parseAcceleration(acceleration_, temperature_, msg);
      acceleration_ *= gravity_;
      sensor_updated_ = sensor_updated_ | DataType::kAcceleration | DataType::kTemperature;
      break;
    case std_packet_type::kAngularVelocity:
      standard::parseAngularVelocity(angular_velocity_, voltage_, msg);
      sensor_updated_ = sensor_updated_ | DataType::kAngularVelocity | DataType::kVoltage;
      break;
    case std_packet_type::kAngle:
      standard::parseAngle(angle_, version_, msg);
      sensor_updated_ = sensor_updated_ | DataType::kAngle;
      break;
    case std_packet_type::kMagneticField:
      standard::parseMagneticField(magnetic_field_, msg);
      sensor_updated_ = sensor_updated_ | DataType::kMagneticField;
      break;
    default:
      break;
  }
}

WitmotionSerialImu::DataType operator|(
  WitmotionSerialImu::DataType l, WitmotionSerialImu::DataType r)
{
  const auto rr = std::underlying_type<WitmotionSerialImu::DataType>::type(l);
  const auto ll = std::underlying_type<WitmotionSerialImu::DataType>::type(r);
  return WitmotionSerialImu::DataType(rr | ll);
}

WitmotionSerialImu::DataType operator&(
  WitmotionSerialImu::DataType l, WitmotionSerialImu::DataType r)
{
  const auto rr = std::underlying_type<WitmotionSerialImu::DataType>::type(l);
  const auto ll = std::underlying_type<WitmotionSerialImu::DataType>::type(r);
  return WitmotionSerialImu::DataType(rr & ll);
}

WitmotionSerialImu::DataType operator^(
  WitmotionSerialImu::DataType l, WitmotionSerialImu::DataType r)
{
  const auto rr = std::underlying_type<WitmotionSerialImu::DataType>::type(l);
  const auto ll = std::underlying_type<WitmotionSerialImu::DataType>::type(r);
  return WitmotionSerialImu::DataType(rr ^ ll);
}

WitmotionSerialImu::DataType operator!(WitmotionSerialImu::DataType v)
{
  const auto vv = std::underlying_type<WitmotionSerialImu::DataType>::type(v);
  return WitmotionSerialImu::DataType(!vv);
}
}  // namespace witmotion_imu_driver_core
