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
}  // namespace standard

Eigen::Vector3f WitmotionSerialImu::getAccel()
{
  Eigen::Vector3f acceleration;
  return acceleration;
}

void WitmotionSerialImu::procSerialStream()
{
  if (communication_type_ == CommunicationType::STANDARD) {
    constexpr std::size_t kMsgLength = 11;
    static SerialPort::Message header_msg{standard::kTypeProtocolHeader};
    const auto recived_msg = serial_port_->read(kMsgLength, header_msg);
    const std::uint8_t crc = static_cast<std::uint8_t>(
      std::accumulate(recived_msg.cbegin(), recived_msg.cend() - 1, 0));
    if (crc == recived_msg[standard::kMessageCrcIndex]) {
      parseSerialMsg(recived_msg);
    }
  } else if (communication_type_ == CommunicationType::MODBUS) {
    throw std::logic_error("Not supported Modbus");
  }
}

void WitmotionSerialImu::parseSerialMsg(const SerialPort::Message & msg)
{
  switch (msg[standard::kMessageTypeHeaderIndex]) {
    case standard::kTypeAccelerationHeader:
      break;
    case standard::kTypeAngularVelocityHeader:
      break;
    case standard::kTypeAngleHeader:
      break;
    case standard::kTypeMagneticFieldHeader:
      break;
    default:
      break;
  }
}
}  // namespace witmotion_serial_imu
