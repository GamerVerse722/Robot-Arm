#pragma once

#include <functional>
#include <string>

namespace PROSLogger {
    enum LogLevel {
        DEBUG = 0,
        INFO = 1,
        WARN = 2,
        ERROR = 3
    };

    inline std::string levelToString(LogLevel level) {
        switch (level) {
            case DEBUG: return "Debug";
            case INFO: return "Info";
            case WARN: return "Warn";
            case ERROR: return "Error";
            default: return "Unknown";
        }
    }

    struct LoggerEvent {
        LogLevel level;
        std::string time;
        std::string groupId;
        std::string message;
        std::string formattedMessage;
    };

    typedef std::function<void(const LoggerEvent)> Subscriber;

    class Logger {
        private:
            const std::string id;
        public:
            explicit Logger(std::string loggerId): id(loggerId) {}

            void log(const std::string& message, LogLevel level);

            void debug(const std::string &message);
            void info(const std::string &message);
            void warn(const std::string &message);
            void error(const std::string &message);
    };
}

namespace PROSLogger::Manager {
    extern LogLevel level;
    extern bool consoleEnabled;
    extern std::vector<Subscriber> subscribers;

    void setLevel(LogLevel level);
    LogLevel getLevel();

    void setConsoleEnabled(bool enabled);
    bool isConsoleEnabled();

    void subscribe(const Subscriber& callback);
    void notify(const LoggerEvent& event);
}