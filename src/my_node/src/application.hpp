#pragma once

#include "sensor-flow/src/application.hpp"

namespace application
{

// rclcpp::init()の後で呼ぶ。ROSノード自身を各センサーのdistributorにする。
std::expected<std::unique_ptr<manager::Manager>, error::Error> initRos(const std::string& path);

} // namespace application
