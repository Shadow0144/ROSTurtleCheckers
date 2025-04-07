#include "CheckersGameLobby.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "turtle_checkers_interfaces/srv/request_piece_move.hpp"
#include "turtle_checkers_interfaces/srv/request_reachable_tiles.hpp"
#include "turtle_checkers_interfaces/msg/declare_winner.hpp"
#include "turtle_checkers_interfaces/msg/game_start.hpp"
#include "turtle_checkers_interfaces/msg/player_ready.hpp"
#include "turtle_checkers_interfaces/msg/update_board.hpp"

#include "MasterBoard.hpp"

#include <memory>

using std::placeholders::_1;
using std::placeholders::_2;

CheckersGameLobby::CheckersGameLobby(rclcpp::Node::SharedPtr &nodeHandle, const std::string &lobbyName)
    : m_lobbyName(lobbyName)
{
    m_nodeHandle = nodeHandle;

    m_board = std::make_shared<MasterBoard>();

    m_isBlackTurn = true;

    m_blackPlayerName = "";
    m_redPlayerName = "";
    
    m_blackPlayerReady = false;
    m_redPlayerReady = false;

    m_requestReachableTilesService =
        m_nodeHandle->create_service<turtle_checkers_interfaces::srv::RequestReachableTiles>(
            m_lobbyName + "/RequestReachableTiles", std::bind(&CheckersGameLobby::requestReachableTilesRequest, this, std::placeholders::_1, std::placeholders::_2));
    m_requestPieceMoveService =
        m_nodeHandle->create_service<turtle_checkers_interfaces::srv::RequestPieceMove>(
            m_lobbyName + "/RequestPieceMove", std::bind(&CheckersGameLobby::requestPieceMoveRequest, this, std::placeholders::_1, std::placeholders::_2));

    m_declareWinnerPublisher = m_nodeHandle->create_publisher<turtle_checkers_interfaces::msg::DeclareWinner>(
        m_lobbyName + "/DeclareWinner", 10);
    m_gameStartPublisher = m_nodeHandle->create_publisher<turtle_checkers_interfaces::msg::GameStart>(
        m_lobbyName + "/GameStart", 10);
    m_updateBoardPublisher = m_nodeHandle->create_publisher<turtle_checkers_interfaces::msg::UpdateBoard>(
        m_lobbyName + "/UpdateBoard", 10);

    m_playerReadySubscription = m_nodeHandle->create_subscription<turtle_checkers_interfaces::msg::PlayerReady>(
        m_lobbyName + "/PlayerReady", 10, std::bind(&CheckersGameLobby::playerReadyCallback, this, std::placeholders::_1));

    RCLCPP_INFO(m_nodeHandle->get_logger(), "Starting Turtles Checkers game node; now accepting players!");
}

bool CheckersGameLobby::playerSlotAvailable() const
{
    return (m_blackPlayerName.empty() || m_redPlayerName.empty());
}

bool CheckersGameLobby::containsPlayer(const std::string &playerName) const
{
    return ((playerName == m_blackPlayerName) || (playerName == m_redPlayerName));
}

TurtlePieceColor CheckersGameLobby::addPlayer(const std::string &playerName)
{
    if (m_blackPlayerName.empty())
    {
        m_blackPlayerName = playerName;
        return TurtlePieceColor::Black;
    }
    else if (m_redPlayerName.empty())
    {
        m_redPlayerName = playerName;
        return TurtlePieceColor::Red;
    }
    else
    {
        return TurtlePieceColor::None;
    }
}

void CheckersGameLobby::setPlayerReady(const std::string &playerName)
{
    if (m_blackPlayerName == playerName)
    {
        m_blackPlayerReady = true;
    }
    else if (m_redPlayerName == playerName)
    {
        m_redPlayerReady = true;
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
    setPlayerReady(message->player_name);

    if (getAreAllPlayersReady())
    {
        auto startMessage = turtle_checkers_interfaces::msg::GameStart();
        startMessage.lobby_name = message->lobby_name;
        startMessage.game_state = 2; // Black to move
        m_board->checkPlayersCanMove(m_isBlackTurn, startMessage.movable_tile_indices);
        m_gameStartPublisher->publish(startMessage);
        RCLCPP_INFO(m_nodeHandle->get_logger(), "Starting game!");
    }
}