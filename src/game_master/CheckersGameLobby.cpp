#include "game_master/CheckersGameLobby.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "turtle_checkers_interfaces/msg/chat_message.hpp"
#include "turtle_checkers_interfaces/msg/declare_winner.hpp"
#include "turtle_checkers_interfaces/msg/draw_declined.hpp"
#include "turtle_checkers_interfaces/msg/draw_offered.hpp"
#include "turtle_checkers_interfaces/msg/forfeit.hpp"
#include "turtle_checkers_interfaces/msg/game_start.hpp"
#include "turtle_checkers_interfaces/msg/kick_player.hpp"
#include "turtle_checkers_interfaces/msg/offer_draw.hpp"
#include "turtle_checkers_interfaces/msg/player_joined_lobby.hpp"
#include "turtle_checkers_interfaces/msg/player_kicked.hpp"
#include "turtle_checkers_interfaces/msg/player_left_lobby.hpp"
#include "turtle_checkers_interfaces/msg/player_readied.hpp"
#include "turtle_checkers_interfaces/msg/player_ready.hpp"
#include "turtle_checkers_interfaces/msg/timer_changed.hpp"
#include "turtle_checkers_interfaces/msg/update_chat.hpp"
#include "turtle_checkers_interfaces/msg/update_board.hpp"
#include "turtle_checkers_interfaces/msg/update_lobby_owner.hpp"
#include "turtle_checkers_interfaces/msg/update_timer.hpp"
#include "turtle_checkers_interfaces/srv/request_piece_move.hpp"
#include "turtle_checkers_interfaces/srv/request_reachable_tiles.hpp"
#include "turtle_checkers_interfaces/srv/request_board_state.hpp"
#include "turtle_checkers_interfaces/srv/resync_board.hpp"

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <ctime>
#include <random>

#include "shared/Hasher.hpp"
#include "shared/RSAKeyGenerator.hpp"
#include "game_master/MasterBoard.hpp"
#include "game_master/DatabaseHandler.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

CheckersGameLobby::CheckersGameLobby(rclcpp::Node::SharedPtr &nodeHandle,
                                     uint64_t publicKey,
                                     uint64_t privateKey,
                                     const std::string &lobbyName,
                                     const std::string &lobbyId,
                                     uint64_t lobbyPasswordHash,
                                     const std::weak_ptr<DatabaseHandler> &databaseHandler)
    : m_lobbyName(lobbyName),
      m_lobbyId(lobbyId)
{
    m_nodeHandle = nodeHandle;
    m_publicKey = publicKey;
    m_privateKey = privateKey;
    m_lobbyPasswordHash = lobbyPasswordHash;

    m_databaseHandler = databaseHandler;

    m_board = std::make_shared<MasterBoard>();

    std::srand(std::time({}));

    m_gameState = GameState::Connected;

    m_lobbyOwnerPlayerName = "";

    m_blackPlayerName = "";
    m_redPlayerName = "";

    m_blackPlayerReady = false;
    m_redPlayerReady = false;

    m_timer = std::chrono::seconds(0u);
    m_blackTimeRemaining = std::chrono::seconds(0u);
    m_redTimeRemaining = std::chrono::seconds(0u);
    m_timerThreadRunning = false;

    m_blackPlayerPublicKey = 0u;
    m_redPlayerPublicKey = 0u;
    m_playerPublicKeysByColor[TurtlePieceColor::Black] = 0u;
    m_playerPublicKeysByColor[TurtlePieceColor::Red] = 0u;
    m_playerPublicKeysByColor[TurtlePieceColor::None] = 0u;

    m_playerOfferingDraw = "";
    m_requestReachableTilesService =
        m_nodeHandle->create_service<turtle_checkers_interfaces::srv::RequestReachableTiles>(
            m_lobbyName + "/id" + m_lobbyId + "/RequestReachableTiles", std::bind(&CheckersGameLobby::requestReachableTilesRequest, this, std::placeholders::_1, std::placeholders::_2));
    m_requestPieceMoveService =
        m_nodeHandle->create_service<turtle_checkers_interfaces::srv::RequestPieceMove>(
            m_lobbyName + "/id" + m_lobbyId + "/RequestPieceMove", std::bind(&CheckersGameLobby::requestPieceMoveRequest, this, std::placeholders::_1, std::placeholders::_2));
    m_resyncBoardService =
        m_nodeHandle->create_service<turtle_checkers_interfaces::srv::ResyncBoard>(
            m_lobbyName + "/id" + m_lobbyId + "/ResyncBoard", std::bind(&CheckersGameLobby::resyncBoardRequest, this, std::placeholders::_1, std::placeholders::_2));

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
    m_playerKickedPublisher = m_nodeHandle->create_publisher<turtle_checkers_interfaces::msg::PlayerKicked>(
        m_lobbyName + "/id" + m_lobbyId + "/PlayerKicked", 10);
    m_playerLeftLobbyPublisher = m_nodeHandle->create_publisher<turtle_checkers_interfaces::msg::PlayerLeftLobby>(
        m_lobbyName + "/id" + m_lobbyId + "/PlayerLeftLobby", 10);
    m_playerReadiedPublisher = m_nodeHandle->create_publisher<turtle_checkers_interfaces::msg::PlayerReadied>(
        m_lobbyName + "/id" + m_lobbyId + "/PlayerReadied", 10);
    m_updateChatPublisher = m_nodeHandle->create_publisher<turtle_checkers_interfaces::msg::UpdateChat>(
        m_lobbyName + "/id" + m_lobbyId + "/UpdateChat", 10);
    m_updateLobbyOwnerPublisher = m_nodeHandle->create_publisher<turtle_checkers_interfaces::msg::UpdateLobbyOwner>(
        m_lobbyName + "/id" + m_lobbyId + "/UpdateLobbyOwner", 10);
    m_updateTimerPublisher = m_nodeHandle->create_publisher<turtle_checkers_interfaces::msg::UpdateTimer>(
        m_lobbyName + "/id" + m_lobbyId + "/UpdateTimer", 10);

    m_chatMessageSubscription = m_nodeHandle->create_subscription<turtle_checkers_interfaces::msg::ChatMessage>(
        m_lobbyName + "/id" + m_lobbyId + "/ChatMessage", 10, std::bind(&CheckersGameLobby::chatMessageCallback, this, std::placeholders::_1));
    m_forfeitSubscription = m_nodeHandle->create_subscription<turtle_checkers_interfaces::msg::Forfeit>(
        m_lobbyName + "/id" + m_lobbyId + "/Forfeit", 10, std::bind(&CheckersGameLobby::forfeitCallback, this, std::placeholders::_1));
    m_kickPlayerSubscription = m_nodeHandle->create_subscription<turtle_checkers_interfaces::msg::KickPlayer>(
        m_lobbyName + "/id" + m_lobbyId + "/KickPlayer", 10, std::bind(&CheckersGameLobby::kickPlayerCallback, this, std::placeholders::_1));
    m_offerDrawSubscription = m_nodeHandle->create_subscription<turtle_checkers_interfaces::msg::OfferDraw>(
        m_lobbyName + "/id" + m_lobbyId + "/OfferDraw", 10, std::bind(&CheckersGameLobby::offerDrawCallback, this, std::placeholders::_1));
    m_playerReadySubscription = m_nodeHandle->create_subscription<turtle_checkers_interfaces::msg::PlayerReady>(
        m_lobbyName + "/id" + m_lobbyId + "/PlayerReady", 10, std::bind(&CheckersGameLobby::playerReadyCallback, this, std::placeholders::_1));
    m_timerChangedSubscription = m_nodeHandle->create_subscription<turtle_checkers_interfaces::msg::TimerChanged>(
        m_lobbyName + "/id" + m_lobbyId + "/TimerChanged", 10, std::bind(&CheckersGameLobby::timerChangedCallback, this, std::placeholders::_1));

    RCLCPP_INFO(m_nodeHandle->get_logger(), "Creating checkers game lobby!");
}

CheckersGameLobby::~CheckersGameLobby()
{
    m_timerThreadRunning = false;
    if (m_timerThread.joinable())
    {
        m_timerThread.join();
    }
    if (m_gameStartTimerThread.joinable())
    {
        m_gameStartTimerThread.join();
    }
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
        m_playerPublicKeysByColor[TurtlePieceColor::Black] = playerPublicKey;
    }
    break;
    case TurtlePieceColor::Red:
    {
        m_redPlayerName = playerName;
        m_redPlayerPublicKey = playerPublicKey;
        m_playerPublicKeysByColor[TurtlePieceColor::Red] = playerPublicKey;
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

void CheckersGameLobby::removePlayer(const std::string &playerName, bool kick)
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

    if (kick)
    {
        auto kickMessage = turtle_checkers_interfaces::msg::PlayerKicked();
        kickMessage.lobby_name = m_lobbyName;
        kickMessage.lobby_id = m_lobbyId;
        kickMessage.player_name = playerName;
        kickMessage.checksum_sig = RSAKeyGenerator::createChecksumSignature(
            std::hash<turtle_checkers_interfaces::msg::PlayerKicked>{}(kickMessage),
            m_publicKey, m_privateKey);
        m_playerKickedPublisher->publish(kickMessage);
    }
    else
    {
        auto leftMessage = turtle_checkers_interfaces::msg::PlayerLeftLobby();
        leftMessage.lobby_name = m_lobbyName;
        leftMessage.lobby_id = m_lobbyId;
        leftMessage.player_name = playerName;
        leftMessage.checksum_sig = RSAKeyGenerator::createChecksumSignature(
            std::hash<turtle_checkers_interfaces::msg::PlayerLeftLobby>{}(leftMessage),
            m_publicKey, m_privateKey);
        m_playerLeftLobbyPublisher->publish(leftMessage);
    }

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

uint64_t CheckersGameLobby::getTimerSeconds() const
{
    return m_timer.count();
}

bool CheckersGameLobby::passwordMatches(uint32_t lobbyPasswordHash) const
{
    return (m_lobbyPasswordHash == 0u || m_lobbyPasswordHash == lobbyPasswordHash);
}

bool CheckersGameLobby::hasPassword() const
{
    return (m_lobbyPasswordHash > 0u);
}

void CheckersGameLobby::togglePlayerTurn()
{
    m_gameState = (m_gameState == GameState::BlackMove) ? GameState::RedMove : GameState::BlackMove;
}

bool CheckersGameLobby::isPieceValidForTurn(int requestedPieceTileIndex) const
{
    return (m_board->getPieceColorAtTileIndex(requestedPieceTileIndex) ==
            ((m_gameState == GameState::BlackMove) ? TurtlePieceColor::Black : TurtlePieceColor::Red));
}

uint64_t CheckersGameLobby::getPlayerPublicKey(const std::string &playerName) const
{
    uint64_t playerPublicKey = 0u;
    if (playerName == m_blackPlayerName)
    {
        playerPublicKey = m_blackPlayerPublicKey;
    }
    else if (playerName == m_redPlayerName)
    {
        playerPublicKey = m_redPlayerPublicKey;
    }
    return playerPublicKey;
}

uint64_t CheckersGameLobby::getPlayerPublicKey(int tileIndex) const
{
    return m_playerPublicKeysByColor.at(m_board->getPieceColorAtTileIndex(tileIndex));
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
    std::lock_guard<std::mutex> lock(m_timerMutex);

    if (!RSAKeyGenerator::checksumSignatureMatches(
            std::hash<turtle_checkers_interfaces::srv::RequestPieceMove::Request::SharedPtr>{}(request),
            getPlayerPublicKey(request->source_tile_index), request->checksum_sig))
    {
        std::cerr << "Checksum failed" << std::endl;
        return; // Checksum did not match with the public key
    }

    if (!isPieceValidForTurn(request->source_tile_index))
    {
        return;
    }

    bool moveAccepted = m_board->requestPieceMove(request->source_tile_index, request->destination_tile_index);
    response->move_accepted = moveAccepted;
    if (moveAccepted)
    {
        auto timePassed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now() - m_startTurnTimestamp);
        if (m_gameState == GameState::BlackMove)
        {
            m_blackTimeRemaining = (timePassed < m_blackTimeRemaining) ? (m_blackTimeRemaining - timePassed) : std::chrono::seconds(0u);
        }
        else if (m_gameState == GameState::RedMove)
        {
            m_redTimeRemaining = (timePassed < m_redTimeRemaining) ? (m_redTimeRemaining - timePassed) : std::chrono::seconds(0u);
        }
        m_startTurnTimestamp = std::chrono::system_clock::now();

        auto message = turtle_checkers_interfaces::msg::UpdateBoard();
        message.lobby_name = request->lobby_name;
        message.lobby_id = request->lobby_id;
        message.black_time_remaining_seconds = m_blackTimeRemaining.count();
        message.red_time_remaining_seconds = m_redTimeRemaining.count();
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
        m_board->checkPlayersCanMove((m_gameState == GameState::BlackMove), movableTileIndices);
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
            m_timerThreadRunning = false;
            m_gameState = GameState::GameFinished;
            message.game_state = 4; // Game over
            auto winnerMessage = turtle_checkers_interfaces::msg::DeclareWinner();
            winnerMessage.lobby_name = m_lobbyName;
            winnerMessage.lobby_id = m_lobbyId;
            winnerMessage.winner = static_cast<size_t>(winner);
            winnerMessage.checksum_sig = RSAKeyGenerator::createChecksumSignature(
                std::hash<turtle_checkers_interfaces::msg::DeclareWinner>{}(winnerMessage),
                m_publicKey, m_privateKey);
            m_declareWinnerPublisher->publish(winnerMessage);

            if (auto databaseHandler = m_databaseHandler.lock())
            {
                databaseHandler->addMatch(m_lobbyName, m_lobbyId,
                                          m_blackPlayerName, m_redPlayerName,
                                          winnerMessage.winner);
            }
        }
        else if (m_gameState == GameState::BlackMove)
        {
            message.game_state = 2; // Black to move
        }
        else if (m_gameState == GameState::RedMove)
        {
            message.game_state = 3; // Red to move
        }

        // Send a hash of the board to make sure the player has the same board state
        // after updating and everything is still in sync
        message.board_hash = std::hash<MasterBoardPtr>{}(m_board);

        message.checksum_sig = RSAKeyGenerator::createChecksumSignature(
            std::hash<turtle_checkers_interfaces::msg::UpdateBoard>{}(message),
            m_publicKey, m_privateKey);

        m_updateBoardPublisher->publish(message);
    }

    response->checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::srv::RequestPieceMove::Response::SharedPtr>{}(response),
        m_publicKey, m_privateKey);
}

void CheckersGameLobby::resyncBoardRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::ResyncBoard::Request> request,
                                           std::shared_ptr<turtle_checkers_interfaces::srv::ResyncBoard::Response> response)
{
    (void)request; // Unused parameter

    response->lobby_name = m_lobbyName;
    response->lobby_id = m_lobbyId;
    response->black_time_remaining_seconds = m_blackTimeRemaining.count();
    response->red_time_remaining_seconds = m_redTimeRemaining.count();
    response->game_state = static_cast<uint64_t>(m_gameState);

    std::vector<std::string> turtlePieceNamePerTile;
    std::vector<uint64_t> turtlePieceColorPerTile;
    std::vector<bool> turtlePieceIsKingedPerTile;
    m_board->getTileInfo(turtlePieceNamePerTile, turtlePieceColorPerTile, turtlePieceIsKingedPerTile);
    response->turtle_piece_name_per_tile = turtlePieceNamePerTile;
    response->turtle_piece_color_per_tile = turtlePieceColorPerTile;
    response->turtle_piece_is_kinged_per_tile = turtlePieceIsKingedPerTile;

    response->board_hash = std::hash<MasterBoardPtr>{}(m_board);

    response->black_pieces_remaining = m_board->getBlackPiecesRemaining();
    response->red_pieces_remaining = m_board->getRedPiecesRemaining();

    response->checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::srv::ResyncBoard::Response::SharedPtr>{}(response),
        m_publicKey, m_privateKey);
}

void CheckersGameLobby::chatMessageCallback(const turtle_checkers_interfaces::msg::ChatMessage::SharedPtr message)
{
    if (!RSAKeyGenerator::checksumSignatureMatches(
            std::hash<turtle_checkers_interfaces::msg::ChatMessage>{}(*message),
            getPlayerPublicKey(message->player_name), message->checksum_sig))
    {
        std::cerr << "Checksum failed" << std::endl;
        return; // Checksum did not match with the public key
    }

    auto updateChatMessage = turtle_checkers_interfaces::msg::UpdateChat();
    updateChatMessage.lobby_name = m_lobbyName;
    updateChatMessage.lobby_id = m_lobbyId;
    updateChatMessage.player_name = message->player_name;
    updateChatMessage.player_color = message->player_color;
    updateChatMessage.msg = message->msg;
    updateChatMessage.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                                      std::chrono::system_clock::now().time_since_epoch())
                                      .count();
    updateChatMessage.checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::msg::UpdateChat>{}(updateChatMessage),
        m_publicKey, m_privateKey);
    m_updateChatPublisher->publish(updateChatMessage);
}

void CheckersGameLobby::forfeitCallback(const turtle_checkers_interfaces::msg::Forfeit::SharedPtr message)
{
    std::lock_guard<std::mutex> lock(m_timerMutex);

    if (!RSAKeyGenerator::checksumSignatureMatches(
            std::hash<turtle_checkers_interfaces::msg::Forfeit>{}(*message),
            getPlayerPublicKey(message->player_name), message->checksum_sig))
    {
        std::cerr << "Checksum failed" << std::endl;
        return; // Checksum did not match with the public key
    }

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

    m_timerThreadRunning = false;
    m_gameState = GameState::GameFinished;
    auto winnerMessage = turtle_checkers_interfaces::msg::DeclareWinner();
    winnerMessage.lobby_name = m_lobbyName;
    winnerMessage.lobby_id = m_lobbyId;
    winnerMessage.winner = static_cast<size_t>(winner);
    winnerMessage.checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::msg::DeclareWinner>{}(winnerMessage),
        m_publicKey, m_privateKey);
    m_declareWinnerPublisher->publish(winnerMessage);

    if (auto databaseHandler = m_databaseHandler.lock())
    {
        databaseHandler->addMatch(m_lobbyName, m_lobbyId,
                                  m_blackPlayerName, m_redPlayerName,
                                  winnerMessage.winner);
    }
}

void CheckersGameLobby::kickPlayerCallback(const turtle_checkers_interfaces::msg::KickPlayer::SharedPtr message)
{
    if (message->requesting_player_name == m_lobbyOwnerPlayerName)
    {
        removePlayer(message->kick_player_name, true);
    }
}

void CheckersGameLobby::offerDrawCallback(const turtle_checkers_interfaces::msg::OfferDraw::SharedPtr message)
{
    std::lock_guard<std::mutex> lock(m_timerMutex);

    if (!RSAKeyGenerator::checksumSignatureMatches(
            std::hash<turtle_checkers_interfaces::msg::OfferDraw>{}(*message),
            getPlayerPublicKey(message->player_name), message->checksum_sig))
    {
        std::cerr << "Checksum failed" << std::endl;
        return; // Checksum did not match with the public key
    }

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
            m_timerThreadRunning = false;
            m_gameState = GameState::GameFinished;
            auto winnerMessage = turtle_checkers_interfaces::msg::DeclareWinner();
            winnerMessage.lobby_name = m_lobbyName;
            winnerMessage.lobby_id = m_lobbyId;
            winnerMessage.winner = static_cast<size_t>(Winner::Draw);
            winnerMessage.checksum_sig = RSAKeyGenerator::createChecksumSignature(
                std::hash<turtle_checkers_interfaces::msg::DeclareWinner>{}(winnerMessage),
                m_publicKey, m_privateKey);
            m_declareWinnerPublisher->publish(winnerMessage);

            if (auto databaseHandler = m_databaseHandler.lock())
            {
                databaseHandler->addMatch(m_lobbyName, m_lobbyId,
                                          m_blackPlayerName, m_redPlayerName,
                                          winnerMessage.winner);
            }
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
    std::lock_guard<std::mutex> lock(m_timerMutex);

    if (!RSAKeyGenerator::checksumSignatureMatches(
            std::hash<turtle_checkers_interfaces::msg::PlayerReady>{}(*message),
            getPlayerPublicKey(message->player_name), message->checksum_sig))
    {
        std::cerr << "Checksum failed" << std::endl;
        return; // Checksum did not match with the public key
    }

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
        // Start the countdown to start the game
        RCLCPP_INFO(m_nodeHandle->get_logger(), "Players are ready! Begin countdown!");
        if (m_gameStartTimerThread.joinable())
        {
            m_gameStartTimerThread.join();
        }
        m_gameStartTimerThread = std::thread([this]()
                                             {
            std::this_thread::sleep_for(MAX_SECONDS_BEFORE_START);
            startGame(); });
    }
}

void CheckersGameLobby::timerChangedCallback(const turtle_checkers_interfaces::msg::TimerChanged::SharedPtr message)
{
    std::lock_guard<std::mutex> lock(m_timerMutex);

    if (!RSAKeyGenerator::checksumSignatureMatches(
            std::hash<turtle_checkers_interfaces::msg::TimerChanged>{}(*message),
            getPlayerPublicKey(message->player_name), message->checksum_sig))
    {
        std::cerr << "Checksum failed" << std::endl;
        return; // Checksum did not match with the public key
    }

    m_timer = std::chrono::seconds(message->timer_seconds);
    m_blackTimeRemaining = m_timer;
    m_redTimeRemaining = m_timer;

    // Announce the change to the other player as well
    auto updateTimerMessage = turtle_checkers_interfaces::msg::UpdateTimer();
    updateTimerMessage.lobby_name = message->lobby_name;
    updateTimerMessage.lobby_id = message->lobby_id;
    updateTimerMessage.timer_seconds = message->timer_seconds;
    updateTimerMessage.checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::msg::UpdateTimer>{}(updateTimerMessage),
        m_publicKey, m_privateKey);
    m_updateTimerPublisher->publish(updateTimerMessage);
}

void CheckersGameLobby::checkTimers()
{
    // This is run on a separate thread
    while (m_timerThreadRunning)
    {
        std::lock_guard<std::mutex> lock(m_timerMutex);

        auto timePassed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now() - m_startTurnTimestamp);
        if (m_gameState == GameState::BlackMove && timePassed > m_blackTimeRemaining)
        {
            // Black has run out of time
            m_timerThreadRunning = false;
            m_gameState = GameState::GameFinished;
            auto winnerMessage = turtle_checkers_interfaces::msg::DeclareWinner();
            winnerMessage.lobby_name = m_lobbyName;
            winnerMessage.lobby_id = m_lobbyId;
            winnerMessage.winner = static_cast<size_t>(Winner::Red);
            winnerMessage.checksum_sig = RSAKeyGenerator::createChecksumSignature(
                std::hash<turtle_checkers_interfaces::msg::DeclareWinner>{}(winnerMessage),
                m_publicKey, m_privateKey);
            m_declareWinnerPublisher->publish(winnerMessage);

            if (auto databaseHandler = m_databaseHandler.lock())
            {
                databaseHandler->addMatch(m_lobbyName, m_lobbyId,
                                          m_blackPlayerName, m_redPlayerName,
                                          winnerMessage.winner);
            }
        }
        else if (m_gameState == GameState::RedMove && timePassed > m_redTimeRemaining)
        {
            // Red has run out of time
            m_timerThreadRunning = false;
            m_gameState = GameState::GameFinished;
            auto winnerMessage = turtle_checkers_interfaces::msg::DeclareWinner();
            winnerMessage.lobby_name = m_lobbyName;
            winnerMessage.lobby_id = m_lobbyId;
            winnerMessage.winner = static_cast<size_t>(Winner::Black);
            winnerMessage.checksum_sig = RSAKeyGenerator::createChecksumSignature(
                std::hash<turtle_checkers_interfaces::msg::DeclareWinner>{}(winnerMessage),
                m_publicKey, m_privateKey);
            m_declareWinnerPublisher->publish(winnerMessage);

            if (auto databaseHandler = m_databaseHandler.lock())
            {
                databaseHandler->addMatch(m_lobbyName, m_lobbyId,
                                          m_blackPlayerName, m_redPlayerName,
                                          winnerMessage.winner);
            }
        }
    }
}

void CheckersGameLobby::startGame()
{
    m_startTurnTimestamp = std::chrono::system_clock::now();
    if (m_timer.count() > 0u)
    {
        if (m_timerThread.joinable())
        {
            m_timerThread.join();
        }
        m_timerThreadRunning = true;
        m_timerThread = std::thread(&CheckersGameLobby::checkTimers, this);
    }
    m_gameState = GameState::BlackMove;
    auto startMessage = turtle_checkers_interfaces::msg::GameStart();
    startMessage.lobby_name = m_lobbyName;
    startMessage.lobby_id = m_lobbyId;
    startMessage.game_state = 2; // Black to move
    startMessage.black_player_name = m_blackPlayerName;
    startMessage.red_player_name = m_redPlayerName;
    startMessage.black_time_remaining_seconds = m_blackTimeRemaining.count();
    startMessage.red_time_remaining_seconds = m_redTimeRemaining.count();
    m_board->checkPlayersCanMove((m_gameState == GameState::BlackMove), startMessage.movable_tile_indices);
    startMessage.checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::msg::GameStart>{}(startMessage),
        m_publicKey, m_privateKey);
    m_gameStartPublisher->publish(startMessage);
    RCLCPP_INFO(m_nodeHandle->get_logger(), "Starting game!");
}