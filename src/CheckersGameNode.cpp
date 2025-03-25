#include "CheckersGameNode.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "turtle_checkers_interfaces/srv/request_reachable_tiles.hpp"

#include "CheckersGameLobby.hpp"

#include <memory>

using std::placeholders::_1;
using std::placeholders::_2;

GamePlayerNode::GamePlayerNode()
    : Node("checkers_game_node")
{
    // A game node creates a 2-player game for player nodes to publish their moves to
    // The game node publishes when it is ready for which player's next move, what the last move was, and when a winner is decided

    std::string lobbyName = "";
    std::string blackPlayerName = "";
    std::string redPlayerName = "";
    m_checkersGameLobby = std::make_shared<CheckersGameLobby>(lobbyName, blackPlayerName, redPlayerName);

    m_requestReachableTilesService =
        this->create_service<turtle_checkers_interfaces::srv::RequestReachableTiles>("RequestReachableTiles", std::bind(&GamePlayerNode::requestReachableTilesRequest, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "Starting Turtles Checkers game node; now accepting players!");
}

void GamePlayerNode::requestReachableTilesRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::RequestReachableTiles::Request> request,
                                                  std::shared_ptr<turtle_checkers_interfaces::srv::RequestReachableTiles::Response> response)
{
    response->reachable_tile_indices = m_checkersGameLobby->requestReachableTiles(request->piece_name);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GamePlayerNode>());
    rclcpp::shutdown();
    return 0;
}