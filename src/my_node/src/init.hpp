#pragma once

#include "init.hpp"
#include "manager.hpp"
#include "utility/error.hpp"

#include <expected>

namespace init
{

std::expected<std::unique_ptr<manager::Manager>, error::Error> init(const std::string& path);

}
