#include "CheckersGameNode.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "turtle_checkers_interfaces/srv/connect_to_game.hpp"
#include "turtle_checkers_interfaces/srv/request_reachable_tiles.hpp"
#include "turtle_checkers_interfaces/srv/request_piece_move.hpp"
#include "turtle_checkers_interfaces/msg/update_board.hpp"
#include "turtle_checkers_interfaces/msg/update_game_state.hpp"

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
    m_checkersGameLobby = std::make_shared<CheckersGameLobby>(lobbyName);

    m_connectToGameService =
        this->create_service<turtle_checkers_interfaces::srv::ConnectToGame>("ConnectToGame", std::bind(&GamePlayerNode::connectToGameRequest, this, std::placeholders::_1, std::placeholders::_2));
    m_requestReachableTilesService =
        this->create_service<turtle_checkers_interfaces::srv::RequestReachableTiles>("RequestReachableTiles", std::bind(&GamePlayerNode::requestReachableTilesRequest, this, std::placeholders::_1, std::placeholders::_2));
    m_requestPieceMoveService =
        this->create_service<turtle_checkers_interfaces::srv::RequestPieceMove>("RequestPieceMove", std::bind(&GamePlayerNode::requestPieceMoveRequest, this, std::placeholders::_1, std::placeholders::_2));

    m_updateGameStatePublisher = this->create_publisher<turtle_checkers_interfaces::msg::UpdateGameState>("UpdateGameState", 10);
    m_updateBoardPublisher = this->create_publisher<turtle_checkers_interfaces::msg::UpdateBoard>("UpdateBoard", 10);

    RCLCPP_INFO(get_logger(), "Starting Turtles Checkers game node; now accepting players!");
}

void GamePlayerNode::connectToGameRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::ConnectToGame::Request> request,
                                          std::shared_ptr<turtle_checkers_interfaces::srv::ConnectToGame::Response> response)
{
    response->lobby_name = m_checkersGameLobby->getLobbyName();
    response->player_color = 0;
    if (m_checkersGameLobby->playerSlotAvailable())
    {
        auto playerColor = m_checkersGameLobby->addPlayer(request->player_name);
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
            RCLCPP_INFO(get_logger(), "Black player connected: " + request->player_name + "!");
        }
        break;
        case TurtlePieceColor::Red:
        {
            response->player_color = 2;
            RCLCPP_INFO(get_logger(), "Red player connected: " + request->player_name + "!");
        }
        break;
        }
    }
    if (!m_checkersGameLobby->playerSlotAvailable()) // Game is full, start it
    {
        response->game_state = 2; // Black to move
        auto message = turtle_checkers_interfaces::msg::UpdateGameState();
        message.game_state = 2; // Black to move
        m_updateGameStatePublisher->publish(message);
        RCLCPP_INFO(get_logger(), "Starting game!");
    }
    else
    {
        response->game_state = 1; // Connected
    }
}

void GamePlayerNode::requestReachableTilesRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::RequestReachableTiles::Request> request,
                                                  std::shared_ptr<turtle_checkers_interfaces::srv::RequestReachableTiles::Response> response)
{
    response->reachable_tile_indices = m_checkersGameLobby->requestReachableTiles(request->piece_name);
}

void GamePlayerNode::requestPieceMoveRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::RequestPieceMove::Request> request,
                                             std::shared_ptr<turtle_checkers_interfaces::srv::RequestPieceMove::Response> response)
{
    bool moveAccepted = m_checkersGameLobby->requestPieceMove(request->piece_name, request->source_tile_index, request->destination_tile_index);
    response->move_accepted = moveAccepted;
    if (moveAccepted)
    {
        auto jumpedPieceTileIndex = m_checkersGameLobby->getJumpedPieceTileIndex(request->source_tile_index, request->destination_tile_index);
        if (jumpedPieceTileIndex == -1) // Toggle the player turn if no piece was jumped
        {
            m_checkersGameLobby->togglePlayerTurn();
        }
        else
        {
            m_checkersGameLobby->slayTurtleAtTileIndex(jumpedPieceTileIndex);
        }
        auto message = turtle_checkers_interfaces::msg::UpdateBoard();
        message.piece_name = request->piece_name;
        message.source_tile_index = request->source_tile_index;
        message.destination_tile_index = request->destination_tile_index;
        message.slain_piece_tile_index = jumpedPieceTileIndex;
        if (m_checkersGameLobby->getWinner() != Winner::None)
        {
            message.game_state = 4; // Game over
        }
        else if (m_checkersGameLobby->getIsBlackTurn())
        {
            message.game_state = 2; // Black to move
        }
        else
        {
            message.game_state = 3; // Red to move
        }
        m_updateBoardPublisher->publish(message);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GamePlayerNode>());
    rclcpp::shutdown();
    return 0;
}