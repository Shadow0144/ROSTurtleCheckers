#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <QApplication>

class CheckersBoardNode : public QApplication
{
public:
    explicit CheckersBoardNode(int & argc, char ** argv);

    int exec();

private:
    std::shared_ptr<rclcpp::Node> board_node;
};