#include "commander.hpp"
#include "init.hpp"
#include "utility/logger.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <stop_token>
#include <thread>
#include <utility>

int main(int argc, char** argv)
{
        using namespace std::chrono_literals;

        const auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
        if (args.size() != 2 || args[1].empty()) {
                RCLCPP_ERROR(rclcpp::get_logger("main"),
                             "usage: %s <serial-device> [--ros-args ...]",
                             args[0].c_str());
                rclcpp::shutdown();
                return 1;
        }

        auto initialized = init::init(args[1]);
        if (!initialized) {
                logger::log(initialized.error());
                rclcpp::shutdown();
                return 1;
        }
        auto app       = std::move(initialized.value());
        RCLCPP_INFO(rclcpp::get_logger("main"),
                    "Application initialized: serial=%s",
                    args[1].c_str());
        auto commander = std::make_shared<commander::Commander>(*app->directiveQueue);

        std::stop_source shutdown;
        rclcpp::on_shutdown([shutdown]() mutable { shutdown.request_stop(); });

        auto status = app->receiverWorker.dispatch();
        for (auto& [_, parser] : app->parsers) {
                if (!status)
                        break;
                status = parser.dispatch();
        }
        for (auto& [_, distributor] : app->distributors) {
                if (!status)
                        break;
                status = distributor.dispatch();
        }

        std::stop_callback stopWorkers(shutdown.get_token(), [&] {
                app->receiverWorker.thread.request_stop();
                for (auto& [_, parser] : app->parsers)
                        parser.thread.request_stop();
                for (auto& [_, distributor] : app->distributors)
                        distributor.thread.request_stop();
        });
        // dispatchが完了してから登録し、生成中のthreadとの競合を避ける。

        if (!status) {
                logger::log(status.error());
                rclcpp::shutdown();
                return 1;
        }

        RCLCPP_INFO(rclcpp::get_logger("main"),
                    "Worker dispatch complete: parsers=%zu, distributors=%zu",
                    app->parsers.size(),
                    app->distributors.size());
        // CommanderはWorkerへ入れず、mainがthreadを所有する。
        std::jthread commanderThread([commander](std::stop_token st) { commander->run(st); });

        while (rclcpp::ok())
                std::this_thread::sleep_for(100ms);

        commanderThread.request_stop();
        return 0;
}
