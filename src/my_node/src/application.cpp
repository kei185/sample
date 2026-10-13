#include "application.hpp"
#include "encoder.hpp"
#include "imu.hpp"
#include "lidar.hpp"

namespace application
{

std::expected<std::unique_ptr<manager::Manager>, error::Error> initRos(const std::string& path)
{
        auto result = init(path);
        if (!result)
                return std::unexpected(result.error());

        auto& manager = **result;
        auto& streams = *manager.dataStreams;

        // parserが使うqueueをそのままノードへ渡し、既存のplotterを置き換える。
        manager.distributors[frame::Type::LIDAR]   = worker::Worker(std::make_unique<lidar::Lidar>(
                static_cast<xqueue::Queue<frame::LidarPoint>&>(*streams.at(frame::Type::LIDAR))));
        manager.distributors[frame::Type::IMU]     = worker::Worker(std::make_unique<imu::Imu>(
                static_cast<xqueue::Queue<frame::Imu>&>(*streams.at(frame::Type::IMU))));
        manager.distributors[frame::Type::ENCODER] = worker::Worker(
                std::make_unique<encoder::Encoder>(static_cast<xqueue::Queue<frame::Encoder>&>(
                        *streams.at(frame::Type::ENCODER))));

        return result;
}

} // namespace application
