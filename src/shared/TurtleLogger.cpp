#include "shared/TurtleLogger.hpp"

#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

static std::string s_nodeName;
static std::mutex s_logMutex;

void TurtleLogger::initialize(const std::string &nodeName)
{
    s_nodeName = nodeName;
}

void TurtleLogger::logInfo(const std::string &infoMessage)
{
    std::lock_guard<std::mutex> lock(s_logMutex);
    RCLCPP_INFO(rclcpp::get_logger(s_nodeName), infoMessage);
}

void TurtleLogger::logWarn(const std::string &warnMessage)
{
    std::lock_guard<std::mutex> lock(s_logMutex);
    RCLCPP_WARN(rclcpp::get_logger(s_nodeName), warnMessage);
}

void TurtleLogger::logError(const std::string &errorMessage)
{
    std::lock_guard<std::mutex> lock(s_logMutex);
    RCLCPP_ERROR(rclcpp::get_logger(s_nodeName), errorMessage);
}