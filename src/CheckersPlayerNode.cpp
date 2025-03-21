#include "CheckersPlayerNode.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>

// Parses command line options
char* getCmdOption(char ** begin, char ** end, const std::string & option)
{
    char ** itr = std::find(begin, end, option);
    if (itr != end && ++itr != end)
    {
        return *itr;
    }
    return 0;
}

CheckersPlayerNode::CheckersPlayerNode(int & argc, char ** argv)
    : QApplication(argc, argv)
{
    rclcpp::init(argc, argv);

    player_name = getCmdOption(argv, argv + argc, "-p");
    if (!player_name)
    {
        player_name = getCmdOption(argv, argv + argc, "-player_name");
    }
    if (!player_name)
    {
        std::cerr << "You need to provide a player name with -p or -player_name" << std::endl;
        return;
    }

    // Try to subscribe to a game and connect to a lobby
    std::cout << "Player " << player_name << " searching for lobby..." << std::endl;

    // Black starts
    player_color = TurtlePiece::TurtleColor::Black;
    game_state = CheckersBoardFrame::GameState::SelectPiece;

    player_node = rclcpp::Node::make_shared("checkers_player_node");
}

int CheckersPlayerNode::exec()
{
    checkers_board = std::make_unique<CheckersBoardFrame>(player_node, player_color, game_state);
    checkers_board->show();

    return QApplication::exec();
}

int main(int argc, char **argv)
{
    CheckersPlayerNode node(argc, argv);
    return node.exec();
}