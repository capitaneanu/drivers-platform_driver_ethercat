#include "base-logging/Logging.hpp"

using namespace platform_driver_ethercat;

enum class LogLevel { DEBUG, INFO, WARN, ERROR, FATAL };

inline void log(const LogLevel level, const std::string context, const std::string message)
{
    switch (level)
    {
        case LogLevel::DEBUG:
            LOG_DEBUG_S << context << message;
        break;
        case LogLevel::INFO:
            LOG_INFO_S << context << message;
        break;
        case LogLevel::WARN:
            LOG_WARN_S << context << message;
        break;
        case LogLevel::ERROR:
            LOG_ERROR_S << context << message;
        break;
        case LogLevel::FATAL:
            LOG_FATAL_S << context << message;
        break;
    }
}
