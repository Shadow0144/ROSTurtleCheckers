#pragma once

#include <string>
#include <mutex>

namespace rclcpp
{
    class Logger;
}

class TurtleLogger
{
public:
    static void initialize(const std::string &);

    static void logInfo(const std::string &infoMessage);
    static void logWarn(const std::string &warnMessage);
    static void logError(const std::string &errorMessage);
};