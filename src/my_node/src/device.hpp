#pragma once

#include <chrono>

namespace device
{

// 仮の周期。各センサーの実際の更新頻度に合わせるのは後で行う。
constexpr auto LIDAR_INTERVAL   = std::chrono::milliseconds(100);
constexpr auto IMU_INTERVAL     = std::chrono::milliseconds(100);
constexpr auto ENCODER_INTERVAL = std::chrono::milliseconds(100);
constexpr auto EXECUTOR_TIMEOUT = std::chrono::milliseconds(50);

} // namespace device
