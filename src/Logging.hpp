#pragma once

#ifdef ROCK
#include "base-logging/Logging.hpp"
#elif ROS2
#include "rclcpp/rclcpp.hpp"
#endif

using namespace platform_driver_ethercat;

enum class LogLevel { DEBUG, INFO, WARN, ERROR, FATAL };

inline void log(const LogLevel level, const std::string context, const std::string message)
{
    switch (level)
    {
        case LogLevel::DEBUG:
            #ifdef ROCK
            LOG_DEBUG_S << context << message;
            #elif ROS2
            RCLCPP_DEBUG(rclcpp::get_logger(context), "%s", message.c_str());
            #endif
            break;
        case LogLevel::INFO:
            #ifdef ROCK
            LOG_INFO_S << context << message;
            #elif ROS2
            RCLCPP_INFO(rclcpp::get_logger(context), "%s", message.c_str());
            #endif
            break;
        case LogLevel::WARN:
            #ifdef ROCK
            LOG_WARN_S << context << message;
            #elif ROS2
            RCLCPP_WARN(rclcpp::get_logger(context), "%s", message.c_str());
            #endif
            break;
        case LogLevel::ERROR:
            #ifdef ROCK
            LOG_ERROR_S << context << message;
            #elif ROS2
            RCLCPP_ERROR(rclcpp::get_logger(context), "%s", message.c_str());
            #endif
            break;
        case LogLevel::FATAL:
            #ifdef ROCK
            LOG_FATAL_S << context << message;
            #elif ROS2
            RCLCPP_FATAL(rclcpp::get_logger(context), "%s", message.c_str());
            #endif
            break;
    }
}
