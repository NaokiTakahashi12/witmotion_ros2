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

#include "serial_port.hpp"

#include <stdexcept>

#include <boost/asio/io_context.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/write.hpp>
#include <boost/system/error_code.hpp>

namespace witmotion_imu_driver
{
SerialPort::SerialPort(boost::asio::io_context & io_context, const SerialPortOptions & options)
: options_(options),
  serial_port_(std::make_unique<boost::asio::serial_port>(io_context))
{
  if (options.device_port_name.empty()) {
    throw std::runtime_error("Empty serial port name");
  }
  if (options.baud_rate < 1) {
    throw std::runtime_error("Zero serial baud rate");
  }
}

SerialPort::~SerialPort()
{
  if (serial_port_) {
    if (isOpen()) {
      serial_port_->close();
    }
  }
}

bool SerialPort::isOpen()
{
  return serial_port_->is_open();
}

bool SerialPort::open()
{
  if (isOpen()) {
    serial_port_->close();
  }
  boost::system::error_code error_code;
  serial_port_->open(options_.device_port_name, error_code);

  if (error_code) {
    return true;
  }
  serial_port_->set_option(boost::asio::serial_port_base::baud_rate(options_.baud_rate));
  return false;
}

bool SerialPort::write(const Message & msg)
{
  if (!isOpen()) {
    return true;
  }
  boost::asio::write(*serial_port_, boost::asio::buffer(msg.data(), msg.size()));
  return false;
}

SerialPort::Message SerialPort::read(std::size_t msg_length)
{
  Message msg;
  if (!isOpen()) {
    return msg;
  }
  msg.resize(msg_length);
  unsigned int read_counter = 0;

  while (true) {
    std::uint8_t read_byte = 0;
    boost::asio::read(*serial_port_, boost::asio::buffer(&read_byte, sizeof(std::uint8_t)));
    msg[read_counter] = read_byte;
    read_counter++;

    if (read_counter >= msg_length) {
      break;
    }
  }
  return msg;
}

SerialPort::Message SerialPort::read(std::size_t msg_length, const Message & header)
{
  constexpr unsigned int kMaxContinueCounter = 1024;
  Message msg;
  msg.resize(msg_length);

  if (!isOpen()) {
    return msg;
  }
  unsigned int read_counter = 0;
  unsigned int continue_counter = 0;
  bool found_start = false;

  while (true) {
    if (continue_counter > kMaxContinueCounter) {
      break;
    }
    if (read_counter >= msg_length) {
      break;
    }
    std::uint8_t read_byte = 0;
    boost::asio::read(*serial_port_, boost::asio::buffer(&read_byte, sizeof(std::uint8_t)));
    msg[read_counter] = read_byte;

    if (found_start) {
      read_counter++;
    } else if (read_byte == header[read_counter]) {
      read_counter++;
      if (read_counter >= header.size()) {
        found_start = true;
      }
    } else {
      continue_counter++;
      read_counter = 0;
    }
  }
  return msg;
}
}  // namespace witmotion_imu_driver
