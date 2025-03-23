#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <QApplication>

#include "CheckersConsts.hpp"
#include "TurtlePiece.hpp"
#include "CheckersBoardFrame.hpp"

class CheckersPlayerNode : public QApplication
{
public:
    explicit CheckersPlayerNode(int & argc, char ** argv);

    int exec();

private:
    std::shared_ptr<rclcpp::Node> player_node;

    CheckersBoardFrameUniPtr checkers_board;

    char * player_name;
    TurtlePieceColor player_color;
    GameState game_state;
};