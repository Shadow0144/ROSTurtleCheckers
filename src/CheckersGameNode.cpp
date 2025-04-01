#include "CheckersGameNode.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "turtle_checkers_interfaces/srv/connect_to_game.hpp"
#include "turtle_checkers_interfaces/srv/request_piece_move.hpp"
#include "turtle_checkers_interfaces/srv/request_reachable_tiles.hpp"
#include "turtle_checkers_interfaces/msg/declare_winner.hpp"
#include "turtle_checkers_interfaces/msg/game_start.hpp"
#include "turtle_checkers_interfaces/msg/player_ready.hpp"
#include "turtle_checkers_interfaces/msg/update_board.hpp"

#include "CheckersGameLobby.hpp"

#include <memory>

using std::placeholders::_1;
using std::placeholders::_2;

GamePlayerNode::GamePlayerNode()
    : Node("checkers_game_node")
{
    // A game node creates a 2-player game for player nodes to publish their moves to
    // The game node publishes when it is ready for which player's next move, what the last move was, and when a winner is decided

    m_nextLobbyName = c_LOBBY_NAME_PREFIX + std::to_string(m_nextLobbyNumber++);
    m_checkersGameLobbies[m_nextLobbyName] = std::make_shared<CheckersGameLobby>(m_nextLobbyName);

    m_connectToGameService =
        this->create_service<turtle_checkers_interfaces::srv::ConnectToGame>("ConnectToGame", std::bind(&GamePlayerNode::connectToGameRequest, this, std::placeholders::_1, std::placeholders::_2));
    m_requestReachableTilesService =
        this->create_service<turtle_checkers_interfaces::srv::RequestReachableTiles>("RequestReachableTiles", std::bind(&GamePlayerNode::requestReachableTilesRequest, this, std::placeholders::_1, std::placeholders::_2));
    m_requestPieceMoveService =
        this->create_service<turtle_checkers_interfaces::srv::RequestPieceMove>("RequestPieceMove", std::bind(&GamePlayerNode::requestPieceMoveRequest, this, std::placeholders::_1, std::placeholders::_2));

    m_declareWinnerPublisher = this->create_publisher<turtle_checkers_interfaces::msg::DeclareWinner>("DeclareWinner", 10);
    m_gameStartPublisher = this->create_publisher<turtle_checkers_interfaces::msg::GameStart>("GameStart", 10);
    m_updateBoardPublisher = this->create_publisher<turtle_checkers_interfaces::msg::UpdateBoard>("UpdateBoard", 10);

    m_playerReadySubscription = this->create_subscription<turtle_checkers_interfaces::msg::PlayerReady>("PlayerReady", 10, std::bind(&GamePlayerNode::playerReadyCallback, this, _1));

    RCLCPP_INFO(get_logger(), "Starting Turtles Checkers game node; now accepting players!");
}

void GamePlayerNode::connectToGameRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::ConnectToGame::Request> request,
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

    if (!checkersGameLobby->playerSlotAvailable()) // The game just filled, start it and open a new lobby
    {
        // Create the next lobby
        m_nextLobbyName = c_LOBBY_NAME_PREFIX + std::to_string(m_nextLobbyNumber++);
        m_checkersGameLobbies[m_nextLobbyName] = std::make_shared<CheckersGameLobby>(m_nextLobbyName);
    }
}

void GamePlayerNode::requestReachableTilesRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::RequestReachableTiles::Request> request,
                                                  std::shared_ptr<turtle_checkers_interfaces::srv::RequestReachableTiles::Response> response)
{
    if (m_checkersGameLobbies.find(request->lobby_name) == m_checkersGameLobbies.end())
    {
        return;
    }

    auto &checkersGameLobby = m_checkersGameLobbies[request->lobby_name];
    response->reachable_tile_indices = checkersGameLobby->requestReachableTiles(request->tile_index);
}

void GamePlayerNode::requestPieceMoveRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::RequestPieceMove::Request> request,
                                             std::shared_ptr<turtle_checkers_interfaces::srv::RequestPieceMove::Response> response)
{
    if (m_checkersGameLobbies.find(request->lobby_name) == m_checkersGameLobbies.end())
    {
        return;
    }

    auto &checkersGameLobby = m_checkersGameLobbies[request->lobby_name];
    bool moveAccepted = checkersGameLobby->requestPieceMove(request->piece_name, request->source_tile_index, request->destination_tile_index);
    response->move_accepted = moveAccepted;
    if (moveAccepted)
    {
        auto message = turtle_checkers_interfaces::msg::UpdateBoard();
        message.lobby_name = request->lobby_name;
        auto jumpedPieceTileIndex = checkersGameLobby->getJumpedPieceTileIndex(request->source_tile_index, request->destination_tile_index);
        if (jumpedPieceTileIndex > -1)
        {
            checkersGameLobby->addTileToJumpedTileIndices(jumpedPieceTileIndex);
            if (checkersGameLobby->canJumpAgainFromTileIndex(request->destination_tile_index))
            {
                checkersGameLobby->setMustJump(true);
                message.mandatory_piece_name = request->piece_name; // Must kill again
            }
            else
            {
                checkersGameLobby->togglePlayerTurn();
                checkersGameLobby->setMustJump(false);
                checkersGameLobby->slayTurtlesAtJumpedTileIndices();
            }
        }
        else
        {
            checkersGameLobby->togglePlayerTurn();
            checkersGameLobby->setMustJump(false);
            checkersGameLobby->slayTurtlesAtJumpedTileIndices();
        }
        message.piece_name = request->piece_name;
        message.source_tile_index = request->source_tile_index;
        message.destination_tile_index = request->destination_tile_index;
        message.king_piece = checkersGameLobby->wasPieceKinged(request->piece_name, request->destination_tile_index);
        message.slain_piece_tile_index = jumpedPieceTileIndex;

        // Check which players (if any) cannot move, which ends the game
        std::vector<size_t> movableTileIndices;
        checkersGameLobby->checkPlayersCanMove(movableTileIndices);
        if (!message.mandatory_piece_name.empty()) // If mid-jump, only the current piece can move
        {
            message.movable_tile_indices = {request->destination_tile_index};
        }
        else // Otherwise, any available piece
        {
            message.movable_tile_indices = movableTileIndices;
        }

        // Decide the next game state
        auto winner = checkersGameLobby->getWinner();
        if (winner != Winner::None)
        {
            message.game_state = 4; // Game over
            auto winnerMessage = turtle_checkers_interfaces::msg::DeclareWinner();
            winnerMessage.winner = static_cast<size_t>(winner);
            m_declareWinnerPublisher->publish(winnerMessage);
        }
        else if (checkersGameLobby->getIsBlackTurn())
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

void GamePlayerNode::playerReadyCallback(const turtle_checkers_interfaces::msg::PlayerReady::SharedPtr message)
{
    if (m_checkersGameLobbies.find(message->lobby_name) == m_checkersGameLobbies.end())
    {
        return;
    }

    auto &checkersGameLobby = m_checkersGameLobbies[message->lobby_name];
    checkersGameLobby->setPlayerReady(message->player_name);

    if (checkersGameLobby->getAreAllPlayersReady())
    {
        auto startMessage = turtle_checkers_interfaces::msg::GameStart();
        startMessage.lobby_name = message->lobby_name;
        startMessage.game_state = 2; // Black to move
        checkersGameLobby->checkPlayersCanMove(startMessage.movable_tile_indices);
        m_gameStartPublisher->publish(startMessage);
        RCLCPP_INFO(get_logger(), "Starting game!");
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GamePlayerNode>());
    rclcpp::shutdown();
    return 0;
}