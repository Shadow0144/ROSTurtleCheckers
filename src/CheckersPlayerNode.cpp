#include "CheckersPlayerNode.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>

// Parses command line options
char *getCmdOption(char **begin, char **end, const std::string &option)
{
    char **itr = std::find(begin, end, option);
    if (itr != end && ++itr != end)
    {
        return *itr;
    }
    return 0;
}

CheckersPlayerNode::CheckersPlayerNode(int &argc, char **argv)
    : QApplication(argc, argv)
{
    rclcpp::init(argc, argv);

    m_playerName = getCmdOption(argv, argv + argc, "-p");
    if (m_playerName.empty())
    {
        m_playerName = getCmdOption(argv, argv + argc, "-player_name");
    }

    m_playerNode = rclcpp::Node::make_shared("checkers_player_node");
}

int CheckersPlayerNode::exec()
{
    if (m_playerName.empty())
    {
        RCLCPP_ERROR(m_playerNode->get_logger(), "You need to provide a player name with -p or -player_name");
        return -1;
    }

    // Try to subscribe to a game and connect to a lobby
    RCLCPP_INFO(m_playerNode->get_logger(), "Player " + m_playerName + " searching for lobby...");

    m_checkersBoard = std::make_unique<CheckersBoardFrame>(m_playerNode, m_playerName);
    m_checkersBoard->show();

    return QApplication::exec();
}

int main(int argc, char **argv)
{
    CheckersPlayerNode node(argc, argv);
    return node.exec();
}