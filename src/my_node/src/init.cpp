#include "init.hpp"
#include "encoder.hpp"
#include "imu.hpp"
#include "lidar.hpp"
#include "core/frame.hpp"
#include "core/xqueue.hpp"
#include "worker/receiver.hpp"
#include "worker/worker.hpp"

#include <expected>
#include <map>
#include <memory>
#include <utility>

namespace init
{

std::expected<std::unique_ptr<manager::Manager>, error::Error> init(const std::string& path)
{
        auto port           = std::make_unique<io::Port>(path);
        auto directiveQueue = std::make_unique<xqueue::Queue<frame::MotorCommand>>();
        auto frameStreams   = std::make_unique<manager::FrameStreams>();
        auto dataStreams    = std::make_unique<manager::DataStreams>();
        auto transmitter    = std::make_unique<transmitter::Transmitter>(*port);

        worker::Worker receiver(std::make_unique<receiver::Receiver>(*port, *frameStreams));

        std::map<frame::Type, worker::Worker> parsers;
        std::map<frame::Type, worker::Worker> distributors;

        auto type = frame::Type::SYSTEM;
        frameStreams->try_emplace(type, std::make_unique<xqueue::Queue<frame::Frame>>());
        auto systemStream  = std::make_unique<xqueue::Queue<frame::systemMessage>>();
        parsers[type]      = worker::worker(type, *frameStreams->at(type), *systemStream);
        distributors[type] = worker::worker(type, *transmitter, *systemStream, *directiveQueue);
        dataStreams->try_emplace(type, std::move(systemStream));

        type = frame::Type::LIDAR;
        frameStreams->try_emplace(type, std::make_unique<xqueue::Queue<frame::Frame>>());
        auto lidarStream = std::make_unique<xqueue::Queue<frame::LidarPoint>>();
        parsers[type]    = worker::worker(type, *frameStreams->at(type), *lidarStream);
        // parserの出力queueをROSノードにも渡す。所有権はdataStreamsへ移す。
        distributors[type] = worker::Worker(std::make_unique<lidar::Lidar>(*lidarStream));
        dataStreams->try_emplace(type, std::move(lidarStream));

        type = frame::Type::IMU;
        frameStreams->try_emplace(type, std::make_unique<xqueue::Queue<frame::Frame>>());
        auto imuStream     = std::make_unique<xqueue::Queue<frame::Imu>>();
        parsers[type]      = worker::worker(type, *frameStreams->at(type), *imuStream);
        distributors[type] = worker::Worker(std::make_unique<imu::Imu>(*imuStream));
        dataStreams->try_emplace(type, std::move(imuStream));

        type = frame::Type::ENCODER;
        frameStreams->try_emplace(type, std::make_unique<xqueue::Queue<frame::Frame>>());
        auto encoderStream = std::make_unique<xqueue::Queue<frame::Encoder>>();
        parsers[type]      = worker::worker(type, *frameStreams->at(type), *encoderStream);
        distributors[type] = worker::Worker(std::make_unique<encoder::Encoder>(*encoderStream));
        dataStreams->try_emplace(type, std::move(encoderStream));

        return std::make_unique<manager::Manager>(std::move(port),
                                                  std::move(directiveQueue),
                                                  std::move(frameStreams),
                                                  std::move(dataStreams),
                                                  std::move(transmitter),
                                                  std::move(receiver),
                                                  std::move(parsers),
                                                  std::move(distributors));
}

} // namespace init
