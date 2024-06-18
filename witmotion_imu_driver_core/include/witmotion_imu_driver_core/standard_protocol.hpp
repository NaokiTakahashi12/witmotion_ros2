#pragma once

namespace witmotion_imu_driver_core
{
namespace packet_type
{
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
}  // namespace packet_type
}  // namespace witmotion_imu_driver_core
