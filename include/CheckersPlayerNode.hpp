#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <QApplication>

class CheckersPlayerNode : public QApplication
{
public:
    explicit CheckersPlayerNode(int & argc, char ** argv);

    int exec();

private:
    std::shared_ptr<rclcpp::Node> player_node;

    char * player_name;
};