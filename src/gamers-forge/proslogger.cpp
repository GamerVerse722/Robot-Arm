#include "gamers-forge/proslogger.hpp"

#include "pros/rtos.hpp"

#include <format>
#include <iostream>


namespace PROSLogger {
    void Logger::log(const std::string& message, LogLevel level) {
        if (level < Manager::getLevel()) return;

        int millis = pros::millis();
        int total_seconds = millis / 1000;

        int milliseconds = millis % 1000;
        int seconds = total_seconds % 60;
        int minutes = total_seconds / 60;

        const std::string time = std::format("{:02}:{:02}:{:03}", minutes, seconds, milliseconds);

        const std::string formatted_message = std::format("[{}] [{}] [{}] {}", time, levelToString(level), id, message);

        if (Manager::isConsoleEnabled()) {
            std::cout << formatted_message << std::endl;
        }

        Manager::notify({
            level,
            time,
            id,
            message,
            formatted_message
        });
    }

    void Logger::debug(const std::string &message) { log(message, DEBUG); }
    void Logger::info(const std::string &message) { log(message, INFO); }
    void Logger::warn(const std::string &message) { log(message, WARN); }
    void Logger::error(const std::string &message) { log(message, ERROR); }
}

namespace PROSLogger::Manager {
    LogLevel level = LogLevel::DEBUG;
    std::vector<Subscriber> subscribers;
    bool consoleEnabled = true;

    void setLevel(LogLevel level) {
        Manager::level = level;
    }

    LogLevel getLevel() {
        return level;
    }

    void setConsoleEnabled(bool enabled) {
        consoleEnabled = enabled;
    }

    bool isConsoleEnabled() {
        return consoleEnabled;
    }

    void subscribe(const Subscriber &callback) {
        subscribers.push_back(callback);
    }

    void notify(const LoggerEvent &event) {
        for (const auto& sub : subscribers) {
            sub(event);
        }
    }
}
