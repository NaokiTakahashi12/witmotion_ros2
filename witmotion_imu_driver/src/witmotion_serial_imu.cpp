#include "witmotion_serial_imu.hpp"

#include <memory>
#include <numeric>

#include "serial_port_options.hpp"
#include "serial_port.hpp"

namespace witmotion_imu_driver
{
WitmotionSerialImu::WitmotionSerialImu(
  boost::asio::io_context & io_context, const SerialPortOptions & options,
  const std::uint8_t device_id)
: options_(options),
  communication_type_(CommunicationType::STANDARD),
  device_id_(device_id),
  gravity_(9.82F),
  serial_port_(std::make_unique<SerialPort>(io_context, options)),
  acceleration_updated_(false),
  angular_velocity_updated_(false),
  angle_updated_(false),
  magnetic_field_updated_(false)
{
  if (connect()) {
    throw std::runtime_error("Failed connect serial port");
  }
}

bool WitmotionSerialImu::connect()
{
  return serial_port_->open();
}

bool WitmotionSerialImu::isConnected()
{
  return serial_port_->isOpen();
}

void WitmotionSerialImu::changeDeviceId(const std::uint8_t device_id)
{
  device_id_ = device_id;
}

std::uint8_t WitmotionSerialImu::getDeviceId() const
{
  return device_id_;
}

namespace standard
{
constexpr unsigned int kMessageHeaderIndex = 0;
constexpr unsigned int kMessageTypeHeaderIndex = 1;
constexpr unsigned int kMessageCrcIndex = 10;
constexpr std::size_t kMessageHeaderSize = 2;
constexpr std::uint8_t kTypeProtocolHeader = 0x55;
constexpr std::uint8_t kTypeAccelerationHeader = 0x51;
constexpr std::uint8_t kTypeAngularVelocityHeader = 0x52;
constexpr std::uint8_t kTypeAngleHeader = 0x53;
constexpr std::uint8_t kTypeMagneticFieldHeader = 0x54;

const SerialPort::Message getRequestUnlock()
{
  return {0xFF, 0xAA, 0x64, 0x88, 0xB5};
}

const SerialPort::Message getRequestSave()
{
  return {0xFF, 0xAA, 0x00, 0x00, 0x00};
}

inline float convertDoubleByteToFloat(const std::uint8_t a, const std::uint8_t b)
{
  std::uint16_t c = (a << 8) | b;
  short ret;
  std::memcpy(&ret, &c, sizeof(short));
  return static_cast<float>(ret);
}

inline Eigen::Vector3f parseVector3(const SerialPort::Message & msg)
{
  constexpr unsigned int kVectorDimention = 3;
  Eigen::Vector3f vec;
  for (unsigned int i = 0; i < kVectorDimention; ++i) {
    const unsigned int head = 2 * i + kMessageHeaderSize;
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

inline void parseMagneticField(
  Eigen::Vector3f & magnetic_field, const SerialPort::Message & msg)
{
  magnetic_field = parseVector3(msg);
}
}  // namespace standard

const Eigen::Vector3f & WitmotionSerialImu::getAcceleleration()
{
  acceleration_updated_ = false;
  return acceleration_;
}

const Eigen::Vector3f & WitmotionSerialImu::getAngularVelocity()
{
  angular_velocity_updated_ = false;
  return angular_velocity_;
}

const Eigen::Vector3f & WitmotionSerialImu::getAngle()
{
  angle_updated_ = false;
  return angle_;
}

const Eigen::Vector3f & WitmotionSerialImu::getMagneticField()
{
  magnetic_field_updated_ = false;
  return magnetic_field_;
}

float WitmotionSerialImu::getTemperature()
{
  return temperature_;
}

float WitmotionSerialImu::getVoltage()
{
  return voltage_;
}

bool WitmotionSerialImu::isUpdatedAcceleration() const
{
  return acceleration_updated_;
}

bool WitmotionSerialImu::isUpdatedAngularVelocity() const
{
  return angular_velocity_updated_;
}

bool WitmotionSerialImu::isUpdatedAngle() const
{
  return angle_updated_;
}

bool WitmotionSerialImu::isUpdatedMagneticField() const
{
  return magnetic_field_updated_;
}

void WitmotionSerialImu::procSerialStream()
{
  if (communication_type_ == CommunicationType::STANDARD) {
    constexpr std::size_t kProcMsgs = 4;
    constexpr std::size_t kMsgLength = 11;
    static SerialPort::Message header_msg{standard::kTypeProtocolHeader};
    for (unsigned int i = 0; i < kProcMsgs; ++i) {
      loadSerialMsg(serial_port_->read(kMsgLength, header_msg));
    }
  } else if (communication_type_ == CommunicationType::MODBUS) {
    throw std::logic_error("Not supported Modbus");
  }
}

void WitmotionSerialImu::loadSerialMsg(const SerialPort::Message & msg)
{
  const std::uint8_t crc = std::accumulate(msg.cbegin(), msg.cend() - 1, 0U);
  if (crc != msg[standard::kMessageCrcIndex]) {
    return;
  }
  switch (msg[standard::kMessageTypeHeaderIndex]) {
    case standard::kTypeAccelerationHeader:
      standard::parseAcceleration(acceleration_, temperature_, msg);
      acceleration_ *= gravity_;
      acceleration_updated_ = true;
      break;
    case standard::kTypeAngularVelocityHeader:
      standard::parseAngularVelocity(angular_velocity_, voltage_, msg);
      angular_velocity_updated_ = true;
      break;
    case standard::kTypeAngleHeader:
      standard::parseAngle(angle_, version_, msg);
      angle_updated_ = true;
      break;
    case standard::kTypeMagneticFieldHeader:
      standard::parseMagneticField(magnetic_field_, msg);
      magnetic_field_updated_ = true;
      break;
    default:
      break;
  }
}
}  // namespace witmotion_serial_imu
