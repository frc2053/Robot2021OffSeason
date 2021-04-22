#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <fmt/chrono.h>

class Logger {
public:
    Logger() {
        auto loggerFileSink = std::make_shared<spdlog::sinks::basic_file_sink_st>(fmt::format("robotLog{:%Y-%m-%d-%H-%M-%S}.txt", fmt::localtime(std::chrono::system_clock::now())));
        auto consoleSink = std::make_shared<spdlog::sinks::stdout_color_sink_st>();
        auto sinkList = spdlog::sinks_init_list({loggerFileSink, consoleSink});

        spdlog::set_default_logger(std::make_shared<spdlog::logger>("Robot", sinkList));

        spdlog::flush_every(std::chrono::seconds(5));

        auto swerveLogger = std::make_shared<spdlog::logger>("Swerve", sinkList);

        spdlog::register_logger(swerveLogger);

        spdlog::info("Mr. Robot says hello :)");
    };
private:
};