#include "CheckersGameMasterNode.hpp"

#include "rclcpp/rclcpp.hpp"

#include "turtle_checkers_interfaces/srv/connect_to_game.hpp"

#include "CheckersGameLobby.hpp"

#include <memory>

using std::placeholders::_1;
using std::placeholders::_2;

CheckersGameMasterNode::CheckersGameMasterNode()
{
    // A game node creates a 2-player game for player nodes to publish their moves to
    // The game node publishes when it is ready for which player's next move, what the last move was, and when a winner is decided
    m_gameMasterNode = rclcpp::Node::make_shared("checkers_game_master_node");

    m_nextLobbyNumber = 1u;
    m_nextLobbyName = c_LOBBY_NAME_PREFIX + std::to_string(m_nextLobbyNumber++);
    m_checkersGameLobbies[m_nextLobbyName] = std::make_shared<CheckersGameLobby>(m_gameMasterNode, m_nextLobbyName);

    m_connectToGameService = m_gameMasterNode->create_service<turtle_checkers_interfaces::srv::ConnectToGame>(
        "ConnectToGame", std::bind(&CheckersGameMasterNode::connectToGameRequest,
                                   this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(m_gameMasterNode->get_logger(), "Starting Turtles Checkers game node; now accepting players!");
}

void CheckersGameMasterNode::connectToGameRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::ConnectToGame::Request> request,
                                                  std::shared_ptr<turtle_checkers_interfaces::srv::ConnectToGame::Response> response)
{
    response->lobby_name = m_nextLobbyName;
    response->player_color = 0;

    auto &checkersGameLobby = m_checkersGameLobbies[m_nextLobbyName];
    auto playerColor = checkersGameLobby->addPlayer(request->player_name);
    switch (playerColor)
    {
    case TurtlePieceColor::None:
    {
        response->player_color = 0;
    }
    break;
    case TurtlePieceColor::Black:
    {
        response->player_color = 1;
        RCLCPP_INFO(m_gameMasterNode->get_logger(), "Black player connected: " + request->player_name + "!");
    }
    break;
    case TurtlePieceColor::Red:
    {
        response->player_color = 2;
        RCLCPP_INFO(m_gameMasterNode->get_logger(), "Red player connected: " + request->player_name + "!");
    }
    break;
    }

    if (!checkersGameLobby->playerSlotAvailable()) // The game just filled, start it and open a new lobby
    {
        // Create the next lobby
        m_nextLobbyName = c_LOBBY_NAME_PREFIX + std::to_string(m_nextLobbyNumber++);
        m_checkersGameLobbies[m_nextLobbyName] = std::make_shared<CheckersGameLobby>(m_gameMasterNode, m_nextLobbyName);
    }
}

std::shared_ptr<rclcpp::Node> &CheckersGameMasterNode::getNodeHandle()
{
    return m_gameMasterNode;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto gameMasterNode = std::make_shared<CheckersGameMasterNode>();
    rclcpp::spin(gameMasterNode->getNodeHandle());
    rclcpp::shutdown();
    return 0;
}