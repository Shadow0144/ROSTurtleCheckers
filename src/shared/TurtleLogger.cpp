#include "shared/TurtleLogger.hpp"

#include <string>

#include "rclcpp/rclcpp.hpp"

static std::string s_nodeName;

void TurtleLogger::initialize(const std::string &nodeName)
{
    s_nodeName = nodeName;
}

void TurtleLogger::logInfo(const std::string &infoMessage)
{
    RCLCPP_INFO(rclcpp::get_logger(s_nodeName), infoMessage);
}

void TurtleLogger::logWarn(const std::string &warnMessage)
{
    RCLCPP_WARN(rclcpp::get_logger(s_nodeName), warnMessage);
}

void TurtleLogger::logError(const std::string &errorMessage)
{
    RCLCPP_ERROR(rclcpp::get_logger(s_nodeName), errorMessage);
}