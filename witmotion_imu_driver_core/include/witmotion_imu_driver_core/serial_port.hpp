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

#include <cstdint>
#include <memory>
#include <vector>

#include <boost/asio/io_context.hpp>
#include <boost/asio/serial_port.hpp>

#include "serial_port_options.hpp"

namespace witmotion_imu_driver_core
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
  Message read(std::size_t msg_length);
  /// @retval empty failed serial read
  /// @retval !empty serial read successful
  Message read(std::size_t msg_length, const Message & header);

private:
  const SerialPortOptions options_;

  std::unique_ptr<boost::asio::serial_port> serial_port_;
};
}  // namespace witmotion_imu_driver_core
