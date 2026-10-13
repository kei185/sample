#pragma once

#include "manager.hpp"
#include "utility/error.hpp"

#include <expected>
#include <memory>
#include <string>

namespace init
{

// rclcpp::init()の後で呼ぶ。parserと各ROSノードが同じqueueを参照する。
std::expected<std::unique_ptr<manager::Manager>, error::Error> init(const std::string& path);

} // namespace init
