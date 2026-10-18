#pragma once

#include <numbers>

namespace device
{

constexpr double RADIAN_PER_DEGREE = std::numbers::pi / 180.0;
// TODO 測る
constexpr double MAX_VELOCITY = 0.5;
constexpr double WHEEL_RADIUS = 0.1;
// TODO: 仮値。実機の左右車輪の中心間距離[m]に置き換える。
constexpr double WHEEL_SEPARATION       = 0.3;
constexpr double ENCODER_SAMPLE_RATE_HZ = 10.0;
const double VELOCITY_PER_ROTATION = 2.0 * std::numbers::pi * WHEEL_RADIUS * ENCODER_SAMPLE_RATE_HZ;

constexpr char ODOM_FRAME_ID[] = "odom";
constexpr char BASE_FRAME_ID[] = "base_link";

constexpr char IMU_FRAME_ID[]   = "imu_link";
constexpr char LIDAR_FRAME_ID[] = "lidar_link";

constexpr float  LIDAR_RANGE_MIN_M   = 0.05f;
constexpr float  LIDAR_RANGE_MAX_M   = 12.0f;
constexpr double LIDAR_METERS_PER_MM = 1.0e-3;

// LSM6DSV16Xの初期設定（加速度±2g・角速度±125dps）に合わせる。
// 0.061 mg/LSBをm/s²へ、4.375 mdps/LSBをrad/sへ変換する。
// TODO 確認する
constexpr double IMU_ACCELERATION_MPS2_PER_LSB      = 0.061e-3 * 9.80665;
constexpr double IMU_ANGULAR_VELOCITY_RADPS_PER_LSB = 4.375e-3 * std::numbers::pi / 180.0;

} // namespace device
