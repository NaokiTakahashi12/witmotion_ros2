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

namespace witmotion_imu_driver_core
{
namespace std_packet_size
{
  constexpr std::size_t kMsgHeader = 2;
  constexpr std::size_t kStreamMsg = 11;
}  // namespace std_packet_size
namespace std_packet_index
{
  constexpr unsigned int kMsgType = 1;
  constexpr unsigned int kMsgCrc = 10;
}  // namespace std_packet_index

namespace std_packet_type
{
  constexpr uint8_t kStdHeader = 0x55;

  constexpr uint8_t kTime = 0x50;
  constexpr uint8_t kAcceleration = 0x51;
  constexpr uint8_t kAngularVelocity = 0x52;
  constexpr uint8_t kAngle = 0x53;
  constexpr uint8_t kMagneticField = 0x54;
  constexpr uint8_t kPort = 0x55;
  constexpr uint8_t kBarometricAltitude = 0x56;
  constexpr uint8_t kLatitudeLongitude = 0x57;
  constexpr uint8_t kGroundSpeed = 0x58;
  constexpr uint8_t kQuaternion = 0x59;
  constexpr uint8_t kGpsLocationAccuracy = 0x5A;
  constexpr uint8_t kRead = 0x5F;
}  // namespace std_packet_type
}  // namespace witmotion_imu_driver_core
