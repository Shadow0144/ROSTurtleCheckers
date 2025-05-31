#include "game_master/CheckersGameLobby.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "turtle_checkers_interfaces/srv/connect_to_game_master.hpp"
#include "turtle_checkers_interfaces/srv/request_piece_move.hpp"
#include "turtle_checkers_interfaces/srv/request_reachable_tiles.hpp"
#include "turtle_checkers_interfaces/msg/declare_winner.hpp"
#include "turtle_checkers_interfaces/msg/draw_declined.hpp"
#include "turtle_checkers_interfaces/msg/draw_offered.hpp"
#include "turtle_checkers_interfaces/msg/forfit.hpp"
#include "turtle_checkers_interfaces/msg/game_start.hpp"
#include "turtle_checkers_interfaces/msg/kick_player.hpp"
#include "turtle_checkers_interfaces/msg/offer_draw.hpp"
#include "turtle_checkers_interfaces/msg/player_joined_lobby.hpp"
#include "turtle_checkers_interfaces/msg/player_left_lobby.hpp"
#include "turtle_checkers_interfaces/msg/player_readied.hpp"
#include "turtle_checkers_interfaces/msg/player_ready.hpp"
#include "turtle_checkers_interfaces/msg/update_board.hpp"
#include "turtle_checkers_interfaces/msg/update_lobby_owner.hpp"

#include <ctime>
#include <memory>
#include <random>

#include "game_master/MasterBoard.hpp"
#include "shared/Hasher.hpp"
#include "shared/RSAKeyGenerator.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

CheckersGameLobby::CheckersGameLobby(rclcpp::Node::SharedPtr &nodeHandle,
                                     uint64_t publicKey,
                                     uint64_t privateKey,
                                     const std::string &lobbyName,
                                     const std::string &lobbyId,
                                     uint64_t lobbyPasswordHash)
    : m_lobbyName(lobbyName),
      m_lobbyId(lobbyId)
{
    m_nodeHandle = nodeHandle;
    m_publicKey = publicKey;
    m_privateKey = privateKey;
    m_lobbyPasswordHash = lobbyPasswordHash;

    m_board = std::make_shared<MasterBoard>();

    std::srand(std::time({}));

    m_isBlackTurn = true;

    m_lobbyOwnerPlayerName = "";

    m_blackPlayerName = "";
    m_redPlayerName = "";

    m_blackPlayerReady = false;
    m_redPlayerReady = false;

    m_playerOfferingDraw = "";

    m_requestReachableTilesService =
        m_nodeHandle->create_service<turtle_checkers_interfaces::srv::RequestReachableTiles>(
            m_lobbyName + "/id" + m_lobbyId + "/RequestReachableTiles", std::bind(&CheckersGameLobby::requestReachableTilesRequest, this, std::placeholders::_1, std::placeholders::_2));
    m_requestPieceMoveService =
        m_nodeHandle->create_service<turtle_checkers_interfaces::srv::RequestPieceMove>(
            m_lobbyName + "/id" + m_lobbyId + "/RequestPieceMove", std::bind(&CheckersGameLobby::requestPieceMoveRequest, this, std::placeholders::_1, std::placeholders::_2));

    m_declareWinnerPublisher = m_nodeHandle->create_publisher<turtle_checkers_interfaces::msg::DeclareWinner>(
        m_lobbyName + "/id" + m_lobbyId + "/DeclareWinner", 10);
    m_drawDeclinedPublisher = m_nodeHandle->create_publisher<turtle_checkers_interfaces::msg::DrawDeclined>(
        m_lobbyName + "/id" + m_lobbyId + "/DrawDeclined", 10);
    m_drawOfferedPublisher = m_nodeHandle->create_publisher<turtle_checkers_interfaces::msg::DrawOffered>(
        m_lobbyName + "/id" + m_lobbyId + "/DrawOffered", 10);
    m_gameStartPublisher = m_nodeHandle->create_publisher<turtle_checkers_interfaces::msg::GameStart>(
        m_lobbyName + "/id" + m_lobbyId + "/GameStart", 10);
    m_updateBoardPublisher = m_nodeHandle->create_publisher<turtle_checkers_interfaces::msg::UpdateBoard>(
        m_lobbyName + "/id" + m_lobbyId + "/UpdateBoard", 10);
    m_playerJoinedLobbyPublisher = m_nodeHandle->create_publisher<turtle_checkers_interfaces::msg::PlayerJoinedLobby>(
        m_lobbyName + "/id" + m_lobbyId + "/PlayerJoinedLobby", 10);
    m_playerLeftLobbyPublisher = m_nodeHandle->create_publisher<turtle_checkers_interfaces::msg::PlayerLeftLobby>(
        m_lobbyName + "/id" + m_lobbyId + "/PlayerLeftLobby", 10);
    m_playerReadiedPublisher = m_nodeHandle->create_publisher<turtle_checkers_interfaces::msg::PlayerReadied>(
        m_lobbyName + "/id" + m_lobbyId + "/PlayerReadied", 10);
    m_updateLobbyOwnerPublisher = m_nodeHandle->create_publisher<turtle_checkers_interfaces::msg::UpdateLobbyOwner>(
        m_lobbyName + "/id" + m_lobbyId + "/UpdateLobbyOwner", 10);

    m_forfitSubscription = m_nodeHandle->create_subscription<turtle_checkers_interfaces::msg::Forfit>(
        m_lobbyName + "/id" + m_lobbyId + "/Forfit", 10, std::bind(&CheckersGameLobby::forfitCallback, this, std::placeholders::_1));
    m_kickPlayerSubscription = m_nodeHandle->create_subscription<turtle_checkers_interfaces::msg::KickPlayer>(
        m_lobbyName + "/id" + m_lobbyId + "/KickPlayer", 10, std::bind(&CheckersGameLobby::kickPlayerCallback, this, std::placeholders::_1));
    m_offerDrawSubscription = m_nodeHandle->create_subscription<turtle_checkers_interfaces::msg::OfferDraw>(
        m_lobbyName + "/id" + m_lobbyId + "/OfferDraw", 10, std::bind(&CheckersGameLobby::offerDrawCallback, this, std::placeholders::_1));
    m_playerReadySubscription = m_nodeHandle->create_subscription<turtle_checkers_interfaces::msg::PlayerReady>(
        m_lobbyName + "/id" + m_lobbyId + "/PlayerReady", 10, std::bind(&CheckersGameLobby::playerReadyCallback, this, std::placeholders::_1));

    RCLCPP_INFO(m_nodeHandle->get_logger(), "Creating checkers game lobby!");
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

void CheckersGameLobby::setLobbyOwner(const std::string &playerName)
{
    m_lobbyOwnerPlayerName = playerName;
}

const std::string &CheckersGameLobby::getLobbyOwner() const
{
    return m_lobbyOwnerPlayerName;
}

TurtlePieceColor CheckersGameLobby::addPlayer(const std::string &playerName, uint64_t playerPublicKey, TurtlePieceColor desiredColor)
{
    TurtlePieceColor acceptedColor = TurtlePieceColor::None;
    if (desiredColor == TurtlePieceColor::Black && m_blackPlayerName.empty())
    {
        acceptedColor = TurtlePieceColor::Black;
    }
    else if (desiredColor == TurtlePieceColor::Red && m_redPlayerName.empty())
    {
        acceptedColor = TurtlePieceColor::Red;
    }
    else if (m_blackPlayerName.empty() && m_redPlayerName.empty())
    {
        if (std::rand() % 2 == 0)
        {
            acceptedColor = TurtlePieceColor::Black;
        }
        else
        {
            acceptedColor = TurtlePieceColor::Red;
        }
    }
    else if (m_blackPlayerName.empty())
    {
        acceptedColor = TurtlePieceColor::Black;
    }
    else if (m_redPlayerName.empty())
    {
        acceptedColor = TurtlePieceColor::Red;
    }
    else
    {
        // Do nothing
    }

    switch (acceptedColor)
    {
    case TurtlePieceColor::Black:
    {
        m_blackPlayerName = playerName;
        m_blackPlayerPublicKey = playerPublicKey;
    }
    break;
    case TurtlePieceColor::Red:
    {
        m_redPlayerName = playerName;
        m_redPlayerPublicKey = playerPublicKey;
    }
    break;
    case TurtlePieceColor::None:
    {
        // Do nothing
    }
    break;
    }

    auto message = turtle_checkers_interfaces::msg::PlayerJoinedLobby();
    message.lobby_name = m_lobbyName;
    message.lobby_id = m_lobbyId;
    message.player_name = playerName;
    message.player_color = static_cast<size_t>(acceptedColor);
    message.checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::msg::PlayerJoinedLobby>{}(message),
        m_publicKey, m_privateKey);
    m_playerJoinedLobbyPublisher->publish(message);

    return acceptedColor;
}

void CheckersGameLobby::removePlayer(const std::string &playerName)
{
    m_lobbyOwnerPlayerName = "";
    if (playerName == m_blackPlayerName)
    {
        m_blackPlayerName.clear();
        m_blackPlayerReady = false;
        m_lobbyOwnerPlayerName = m_redPlayerName;
    }
    else if (playerName == m_redPlayerName)
    {
        m_redPlayerName.clear();
        m_redPlayerReady = false;
        m_lobbyOwnerPlayerName = m_blackPlayerName;
    }

    auto message = turtle_checkers_interfaces::msg::PlayerLeftLobby();
    message.lobby_name = m_lobbyName;
    message.lobby_id = m_lobbyId;
    message.player_name = playerName;
    message.checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::msg::PlayerLeftLobby>{}(message),
        m_publicKey, m_privateKey);
    m_playerLeftLobbyPublisher->publish(message);

    // Also update the lobby owner
    if (!m_lobbyOwnerPlayerName.empty())
    {
        auto message = turtle_checkers_interfaces::msg::UpdateLobbyOwner();
        message.lobby_name = m_lobbyName;
        message.lobby_id = m_lobbyId;
        message.lobby_owner_player_name = m_lobbyOwnerPlayerName;
        message.checksum_sig = RSAKeyGenerator::createChecksumSignature(
            std::hash<turtle_checkers_interfaces::msg::UpdateLobbyOwner>{}(message),
            m_publicKey, m_privateKey);
        m_updateLobbyOwnerPublisher->publish(message);
    }
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

bool CheckersGameLobby::passwordMatches(uint32_t lobbyPasswordHash) const
{
    return (m_lobbyPasswordHash == 0u || m_lobbyPasswordHash == lobbyPasswordHash);
}

bool CheckersGameLobby::hasPassword() const
{
    return (m_lobbyPasswordHash > 0u);
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

    response->checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::srv::RequestReachableTiles::Response::SharedPtr>{}(response),
        m_publicKey, m_privateKey);
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
            winnerMessage.checksum_sig = RSAKeyGenerator::createChecksumSignature(
                std::hash<turtle_checkers_interfaces::msg::DeclareWinner>{}(winnerMessage),
                m_publicKey, m_privateKey);
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

        message.checksum_sig = RSAKeyGenerator::createChecksumSignature(
            std::hash<turtle_checkers_interfaces::msg::UpdateBoard>{}(message),
            m_publicKey, m_privateKey);

        m_updateBoardPublisher->publish(message);
    }

    response->checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::srv::RequestPieceMove::Response::SharedPtr>{}(response),
        m_publicKey, m_privateKey);
}

void CheckersGameLobby::forfitCallback(const turtle_checkers_interfaces::msg::Forfit::SharedPtr message)
{
    Winner winner = Winner::None;
    if (message->player_name == m_blackPlayerName)
    {
        winner = Winner::Red;
    }
    else if (message->player_name == m_redPlayerName)
    {
        winner = Winner::Black;
    }
    else
    {
        return;
    }

    auto winnerMessage = turtle_checkers_interfaces::msg::DeclareWinner();
    winnerMessage.lobby_name = m_lobbyName;
    winnerMessage.lobby_id = m_lobbyId;
    winnerMessage.winner = static_cast<size_t>(winner);
    winnerMessage.checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::msg::DeclareWinner>{}(winnerMessage),
        m_publicKey, m_privateKey);
    m_declareWinnerPublisher->publish(winnerMessage);
}

void CheckersGameLobby::kickPlayerCallback(const turtle_checkers_interfaces::msg::KickPlayer::SharedPtr message)
{
    if (message->requesting_player_name == m_lobbyOwnerPlayerName)
    {
        removePlayer(message->kick_player_name);
    }
}

void CheckersGameLobby::offerDrawCallback(const turtle_checkers_interfaces::msg::OfferDraw::SharedPtr message)
{
    if (message->accept_draw)
    {
        // Either requesting or accepting a draw
        auto drawOfferedMessage = turtle_checkers_interfaces::msg::DrawOffered();
        drawOfferedMessage.lobby_name = message->lobby_name;
        drawOfferedMessage.lobby_id = message->lobby_id;
        drawOfferedMessage.player_name = message->player_name;
        drawOfferedMessage.checksum_sig = RSAKeyGenerator::createChecksumSignature(
            std::hash<turtle_checkers_interfaces::msg::DrawOffered>{}(drawOfferedMessage),
            m_publicKey, m_privateKey);
        m_drawOfferedPublisher->publish(drawOfferedMessage);

        if (!m_playerOfferingDraw.empty() && m_playerOfferingDraw != message->player_name)
        {
            // Both have accepted, create a draw
            auto winnerMessage = turtle_checkers_interfaces::msg::DeclareWinner();
            winnerMessage.lobby_name = m_lobbyName;
            winnerMessage.lobby_id = m_lobbyId;
            winnerMessage.winner = static_cast<size_t>(Winner::Draw);
            winnerMessage.checksum_sig = RSAKeyGenerator::createChecksumSignature(
                std::hash<turtle_checkers_interfaces::msg::DeclareWinner>{}(winnerMessage),
                m_publicKey, m_privateKey);
            m_declareWinnerPublisher->publish(winnerMessage);
        }

        m_playerOfferingDraw = message->player_name;
    }
    else
    {
        auto drawDeclinedMessage = turtle_checkers_interfaces::msg::DrawDeclined();
        drawDeclinedMessage.lobby_name = message->lobby_name;
        drawDeclinedMessage.lobby_id = message->lobby_id;
        drawDeclinedMessage.player_name = message->player_name;
        drawDeclinedMessage.checksum_sig = RSAKeyGenerator::createChecksumSignature(
            std::hash<turtle_checkers_interfaces::msg::DrawDeclined>{}(drawDeclinedMessage),
            m_publicKey, m_privateKey);
        m_drawDeclinedPublisher->publish(drawDeclinedMessage);

        m_playerOfferingDraw.clear();
    }
}

void CheckersGameLobby::playerReadyCallback(const turtle_checkers_interfaces::msg::PlayerReady::SharedPtr message)
{
    setPlayerReady(message->player_name, message->ready);

    // Announce the change to the other player as well
    auto playerReadiedMessage = turtle_checkers_interfaces::msg::PlayerReadied();
    playerReadiedMessage.lobby_name = message->lobby_name;
    playerReadiedMessage.lobby_id = message->lobby_id;
    playerReadiedMessage.player_name = message->player_name;
    playerReadiedMessage.ready = message->ready;
    playerReadiedMessage.checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::msg::PlayerReadied>{}(playerReadiedMessage),
        m_publicKey, m_privateKey);
    m_playerReadiedPublisher->publish(playerReadiedMessage);

    if (getAreAllPlayersReady())
    {
        auto startMessage = turtle_checkers_interfaces::msg::GameStart();
        startMessage.lobby_name = message->lobby_name;
        startMessage.lobby_id = message->lobby_id;
        startMessage.game_state = 2; // Black to move
        startMessage.black_player_name = m_blackPlayerName;
        startMessage.red_player_name = m_redPlayerName;
        m_board->checkPlayersCanMove(m_isBlackTurn, startMessage.movable_tile_indices);
        startMessage.checksum_sig = RSAKeyGenerator::createChecksumSignature(
            std::hash<turtle_checkers_interfaces::msg::GameStart>{}(startMessage),
            m_publicKey, m_privateKey);
        m_gameStartPublisher->publish(startMessage);
        RCLCPP_INFO(m_nodeHandle->get_logger(), "Starting game!");
    }
}