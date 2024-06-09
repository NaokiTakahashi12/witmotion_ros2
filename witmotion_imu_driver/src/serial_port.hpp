#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <boost/asio/io_context.hpp>
#include <boost/asio/serial_port.hpp>

#include "serial_port_options.hpp"

namespace witmotion_imu_driver
{
class SerialPort
{
public:
  using Message = std::vector<std::uint8_t>;

  SerialPort() = delete;
  explicit SerialPort(boost::asio::io_context &, const SerialPortOptions &);
  ~SerialPort();

  /// @retval true failed serial open
  /// @retval false serial open successful
  bool isOpen();
  /// @retval true failed serial open
  /// @retval false serial open successful
  bool open();
  /// @retval true failed serial write
  /// @retval false write serial successful
  bool write(const Message &);
  /// @retval empty failed serial read
  /// @retval !empty serial read successful
  Message read(const std::size_t msg_length);
  /// @retval empty failed serial read
  /// @retval !empty serial read successful
  Message read(const std::size_t msg_length, const Message header);

private:
  const SerialPortOptions options_;

  std::unique_ptr<boost::asio::serial_port> serial_port_;
};
}  // namespace witmotion_imu_driver
