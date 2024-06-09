#pragma once

#include <cstdint>
#include <string>

namespace witmotion_imu_driver
{
struct SerialPortOptions
{
  std::string device_port_name;
  unsigned int baud_rate;

  SerialPortOptions()
  : device_port_name("/dev/ttyUSB0"),
    baud_rate(115200)
  {}
};

}  // namespace witmotion_imu_driver
