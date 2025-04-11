#include "game_master/CheckersGameLobby.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "turtle_checkers_interfaces/srv/connect_to_game_master.hpp"
#include "turtle_checkers_interfaces/srv/request_piece_move.hpp"
#include "turtle_checkers_interfaces/srv/request_reachable_tiles.hpp"
#include "turtle_checkers_interfaces/msg/declare_winner.hpp"
#include "turtle_checkers_interfaces/msg/game_start.hpp"
#include "turtle_checkers_interfaces/msg/player_joined_lobby.hpp"
#include "turtle_checkers_interfaces/msg/player_left_lobby.hpp"
#include "turtle_checkers_interfaces/msg/player_ready.hpp"
#include "turtle_checkers_interfaces/msg/update_board.hpp"

#include <ctime>
#include <memory>
#include <random>

#include "game_master/MasterBoard.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

CheckersGameLobby::CheckersGameLobby(rclcpp::Node::SharedPtr &nodeHandle,
                                     const std::string &lobbyName,
                                     const std::string &lobbyId)
    : m_lobbyName(lobbyName),
      m_lobbyId(lobbyId)
{
    m_nodeHandle = nodeHandle;

    m_board = std::make_shared<MasterBoard>();

    std::srand(std::time({}));

    m_isBlackTurn = true;

    m_blackPlayerName = "";
    m_redPlayerName = "";

    m_blackPlayerReady = false;
    m_redPlayerReady = false;

    m_requestReachableTilesService =
        m_nodeHandle->create_service<turtle_checkers_interfaces::srv::RequestReachableTiles>(
            m_lobbyName + "/id" + m_lobbyId + "/RequestReachableTiles", std::bind(&CheckersGameLobby::requestReachableTilesRequest, this, std::placeholders::_1, std::placeholders::_2));
    m_requestPieceMoveService =
        m_nodeHandle->create_service<turtle_checkers_interfaces::srv::RequestPieceMove>(
            m_lobbyName + "/id" + m_lobbyId + "/RequestPieceMove", std::bind(&CheckersGameLobby::requestPieceMoveRequest, this, std::placeholders::_1, std::placeholders::_2));

    m_declareWinnerPublisher = m_nodeHandle->create_publisher<turtle_checkers_interfaces::msg::DeclareWinner>(
        m_lobbyName + "/id" + m_lobbyId + "/DeclareWinner", 10);
    m_gameStartPublisher = m_nodeHandle->create_publisher<turtle_checkers_interfaces::msg::GameStart>(
        m_lobbyName + "/id" + m_lobbyId + "/GameStart", 10);
    m_updateBoardPublisher = m_nodeHandle->create_publisher<turtle_checkers_interfaces::msg::UpdateBoard>(
        m_lobbyName + "/id" + m_lobbyId + "/UpdateBoard", 10);
    m_playerJoinedLobbyPublisher = m_nodeHandle->create_publisher<turtle_checkers_interfaces::msg::PlayerJoinedLobby>(
        m_lobbyName + "/id" + m_lobbyId + "/PlayerJoinedLobby", 10);
    m_playerLeftLobbyPublisher = m_nodeHandle->create_publisher<turtle_checkers_interfaces::msg::PlayerLeftLobby>(
        m_lobbyName + "/id" + m_lobbyId + "/PlayerLeftLobby", 10);

    m_playerReadySubscription = m_nodeHandle->create_subscription<turtle_checkers_interfaces::msg::PlayerReady>(
        m_lobbyName + "/id" + m_lobbyId + "/PlayerReady", 10, std::bind(&CheckersGameLobby::playerReadyCallback, this, std::placeholders::_1));

    RCLCPP_INFO(m_nodeHandle->get_logger(), "Starting Turtles Checkers game node; now accepting players!");
}

const std::string &CheckersGameLobby::getLobbyName() const
{
    return m_lobbyName;
}

const std::string &CheckersGameLobby::getLobbyId() const
{
    return m_lobbyId;
}

bool CheckersGameLobby::isLobbyEmpty() const
{
    return (m_blackPlayerName.empty() && m_redPlayerName.empty());
}

bool CheckersGameLobby::isPlayerSlotAvailable() const
{
    return (m_blackPlayerName.empty() || m_redPlayerName.empty());
}

bool CheckersGameLobby::containsPlayer(const std::string &playerName) const
{
    return ((playerName == m_blackPlayerName) || (playerName == m_redPlayerName));
}

TurtlePieceColor CheckersGameLobby::addPlayer(const std::string &playerName, TurtlePieceColor desiredColor)
{
    TurtlePieceColor acceptedColor = TurtlePieceColor::None;
    if (desiredColor == TurtlePieceColor::Black && m_blackPlayerName.empty())
    {
        m_blackPlayerName = playerName;
    }
    else if (desiredColor == TurtlePieceColor::Red && m_redPlayerName.empty())
    {
        m_redPlayerName = playerName;
    }
    else if (m_blackPlayerName.empty() && m_redPlayerName.empty())
    {
        if (std::rand() % 2 == 0)
        {
            m_blackPlayerName = playerName;
        }
        else
        {
            m_redPlayerName = playerName;
        }
    }
    else if (m_blackPlayerName.empty())
    {
        m_blackPlayerName = playerName;
    }
    else if (m_redPlayerName.empty())
    {
        m_redPlayerName = playerName;
    }
    else
    {
        // Do nothing
    }

    auto message = turtle_checkers_interfaces::msg::PlayerJoinedLobby();
    message.lobby_name = m_lobbyName;
    message.lobby_id = m_lobbyId;
    message.player_name = playerName;
    message.player_color = static_cast<size_t>(TurtlePieceColor::Black);
    m_playerJoinedLobbyPublisher->publish(message);

    return acceptedColor;
}

void CheckersGameLobby::removePlayer(const std::string &playerName)
{
    if (playerName == m_blackPlayerName)
    {
        m_blackPlayerName.clear();
        m_blackPlayerReady = false;
    }
    else if (playerName == m_redPlayerName)
    {
        m_redPlayerName.clear();
        m_redPlayerReady = false;
    }

    auto message = turtle_checkers_interfaces::msg::PlayerLeftLobby();
    message.lobby_name = m_lobbyName;
    message.lobby_id = m_lobbyId;
    message.player_name = playerName;
    m_playerLeftLobbyPublisher->publish(message);
}

const std::string &CheckersGameLobby::getBlackPlayerName() const
{
    return m_blackPlayerName;
}

const std::string &CheckersGameLobby::getRedPlayerName() const
{
    return m_redPlayerName;
}

bool CheckersGameLobby::getBlackPlayerReady() const
{
    return m_blackPlayerReady;
}

bool CheckersGameLobby::getRedPlayerReady() const
{
    return m_redPlayerReady;
}

void CheckersGameLobby::setPlayerReady(const std::string &playerName, bool ready)
{
    if (m_blackPlayerName == playerName)
    {
        m_blackPlayerReady = ready;
    }
    else if (m_redPlayerName == playerName)
    {
        m_redPlayerReady = ready;
    }
}

bool CheckersGameLobby::getAreAllPlayersReady() const
{
    return (m_blackPlayerReady && m_redPlayerReady);
}

void CheckersGameLobby::setIsBlackTurn(bool isBlackTurn)
{
    m_isBlackTurn = isBlackTurn;
}

bool CheckersGameLobby::getIsBlackTurn() const
{
    return m_isBlackTurn;
}

void CheckersGameLobby::togglePlayerTurn()
{
    m_isBlackTurn = !m_isBlackTurn;
}

bool CheckersGameLobby::isPieceValidForTurn(int requestedPieceTileIndex) const
{
    return (m_board->getPieceColorAtTileIndex(requestedPieceTileIndex) ==
            ((m_isBlackTurn) ? TurtlePieceColor::Black : TurtlePieceColor::Red));
}

void CheckersGameLobby::requestReachableTilesRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::RequestReachableTiles::Request> request,
                                                     std::shared_ptr<turtle_checkers_interfaces::srv::RequestReachableTiles::Response> response)
{
    response->reachable_tile_indices = m_board->requestReachableTiles(request->tile_index);
}

void CheckersGameLobby::requestPieceMoveRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::RequestPieceMove::Request> request,
                                                std::shared_ptr<turtle_checkers_interfaces::srv::RequestPieceMove::Response> response)
{
    if (!isPieceValidForTurn(request->source_tile_index))
    {
        return;
    }

    bool moveAccepted = m_board->requestPieceMove(request->source_tile_index, request->destination_tile_index);
    response->move_accepted = moveAccepted;
    if (moveAccepted)
    {
        auto message = turtle_checkers_interfaces::msg::UpdateBoard();
        message.lobby_name = request->lobby_name;
        message.lobby_id = request->lobby_id;
        bool mustContinueJump = false;
        auto jumpedPieceTileIndex = m_board->getJumpedPieceTileIndex(request->source_tile_index, request->destination_tile_index);
        if (jumpedPieceTileIndex > -1)
        {
            m_board->addTileToJumpedTileIndices(jumpedPieceTileIndex);
            if (m_board->canJumpAgainFromTileIndex(request->destination_tile_index))
            {
                m_board->setMustJump(true);
                mustContinueJump = true; // Must kill again
            }
            else
            {
                togglePlayerTurn();
                m_board->setMustJump(false);
                m_board->slayTurtlesAtJumpedTileIndices();
            }
        }
        else
        {
            togglePlayerTurn();
            m_board->setMustJump(false);
            m_board->slayTurtlesAtJumpedTileIndices();
        }
        message.source_tile_index = request->source_tile_index;
        message.destination_tile_index = request->destination_tile_index;
        message.king_piece = m_board->wasPieceKinged(request->destination_tile_index);
        message.slain_piece_tile_index = jumpedPieceTileIndex;

        // Check which players (if any) cannot move, which ends the game
        std::vector<size_t> movableTileIndices;
        m_board->checkPlayersCanMove(m_isBlackTurn, movableTileIndices);
        if (mustContinueJump) // If mid-jump, only the current piece can move
        {
            message.movable_tile_indices = {request->destination_tile_index};
        }
        else // Otherwise, any available piece
        {
            message.movable_tile_indices = movableTileIndices;
        }

        // Decide the next game state
        auto winner = m_board->getWinner();
        if (winner != Winner::None)
        {
            message.game_state = 4; // Game over
            auto winnerMessage = turtle_checkers_interfaces::msg::DeclareWinner();
            winnerMessage.lobby_name = m_lobbyName;
            winnerMessage.lobby_id = m_lobbyId;
            winnerMessage.winner = static_cast<size_t>(winner);
            m_declareWinnerPublisher->publish(winnerMessage);
        }
        else if (getIsBlackTurn())
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

void CheckersGameLobby::playerReadyCallback(const turtle_checkers_interfaces::msg::PlayerReady::SharedPtr message)
{
    setPlayerReady(message->player_name, message->ready);

    if (getAreAllPlayersReady())
    {
        auto startMessage = turtle_checkers_interfaces::msg::GameStart();
        startMessage.lobby_name = message->lobby_name;
        startMessage.lobby_id = message->lobby_id;
        startMessage.game_state = 2; // Black to move
        startMessage.black_player_name = m_blackPlayerName;
        startMessage.red_player_name = m_redPlayerName;
        m_board->checkPlayersCanMove(m_isBlackTurn, startMessage.movable_tile_indices);
        m_gameStartPublisher->publish(startMessage);
        RCLCPP_INFO(m_nodeHandle->get_logger(), "Starting game!");
    }
}