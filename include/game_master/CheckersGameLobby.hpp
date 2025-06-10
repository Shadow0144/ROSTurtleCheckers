#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"

#include "turtle_checkers_interfaces/srv/request_piece_move.hpp"
#include "turtle_checkers_interfaces/srv/request_reachable_tiles.hpp"
#include "turtle_checkers_interfaces/srv/request_board_state.hpp"
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
#include "turtle_checkers_interfaces/msg/timer_changed.hpp"
#include "turtle_checkers_interfaces/msg/update_board.hpp"
#include "turtle_checkers_interfaces/msg/update_lobby_owner.hpp"
#include "turtle_checkers_interfaces/msg/update_timer.hpp"

#include "game_master/MasterBoard.hpp"

class CheckersGameLobby
{
public:
    CheckersGameLobby(rclcpp::Node::SharedPtr &nodeHandle,
                      uint64_t publicKey,
                      uint64_t privateKey,
                      const std::string &lobbyName,
                      const std::string &lobbyId,
                      uint64_t lobbyPasswordHash);
    ~CheckersGameLobby();

    const std::string &getLobbyName() const;
    const std::string &getLobbyId() const;

    bool isLobbyEmpty() const;
    bool isPlayerSlotAvailable() const;
    bool containsPlayer(const std::string &playerName) const;
    TurtlePieceColor addPlayer(const std::string &playerName,
                               uint64_t playerPublicKey,
                               TurtlePieceColor desiredColor);
    void removePlayer(const std::string &playerName);

    void setLobbyOwner(const std::string &playerName);
    const std::string &getLobbyOwner() const;

    const std::string &getBlackPlayerName() const;
    const std::string &getRedPlayerName() const;

    bool getBlackPlayerReady() const;
    bool getRedPlayerReady() const;
    void setPlayerReady(const std::string &playerName, bool ready);
    bool getAreAllPlayersReady() const;

    uint64_t getTimerSeconds() const;

    bool passwordMatches(uint32_t lobbyPasswordHash) const;
    bool hasPassword() const;

    void togglePlayerTurn();

private:
    void requestReachableTilesRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::RequestReachableTiles::Request> request,
                                      std::shared_ptr<turtle_checkers_interfaces::srv::RequestReachableTiles::Response> response);
    void requestPieceMoveRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::RequestPieceMove::Request> request,
                                 std::shared_ptr<turtle_checkers_interfaces::srv::RequestPieceMove::Response> response);
    void requestBoardStateRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::RequestBoardState::Request> request,
                                  std::shared_ptr<turtle_checkers_interfaces::srv::RequestBoardState::Response> response);

    void forfitCallback(const turtle_checkers_interfaces::msg::Forfit::SharedPtr message);
    void kickPlayerCallback(const turtle_checkers_interfaces::msg::KickPlayer::SharedPtr message);
    void offerDrawCallback(const turtle_checkers_interfaces::msg::OfferDraw::SharedPtr message);
    void playerReadyCallback(const turtle_checkers_interfaces::msg::PlayerReady::SharedPtr message);
    void timerChangedCallback(const turtle_checkers_interfaces::msg::TimerChanged::SharedPtr message);

    bool isPieceValidForTurn(int requestedPieceTileIndex) const;

    uint64_t getPlayerPublicKey(const std::string &playerName) const;
    uint64_t getPlayerPublicKey(int tileIndex) const;

    void checkTimers();

    rclcpp::Node::SharedPtr m_nodeHandle;

    rclcpp::Service<turtle_checkers_interfaces::srv::RequestReachableTiles>::SharedPtr m_requestReachableTilesService;
    rclcpp::Service<turtle_checkers_interfaces::srv::RequestPieceMove>::SharedPtr m_requestPieceMoveService;
    rclcpp::Service<turtle_checkers_interfaces::srv::RequestBoardState>::SharedPtr m_requestBoardStateService;

    rclcpp::Publisher<turtle_checkers_interfaces::msg::DeclareWinner>::SharedPtr m_declareWinnerPublisher;
    rclcpp::Publisher<turtle_checkers_interfaces::msg::DrawDeclined>::SharedPtr m_drawDeclinedPublisher;
    rclcpp::Publisher<turtle_checkers_interfaces::msg::DrawOffered>::SharedPtr m_drawOfferedPublisher;
    rclcpp::Publisher<turtle_checkers_interfaces::msg::GameStart>::SharedPtr m_gameStartPublisher;
    rclcpp::Publisher<turtle_checkers_interfaces::msg::UpdateBoard>::SharedPtr m_updateBoardPublisher;
    rclcpp::Publisher<turtle_checkers_interfaces::msg::PlayerJoinedLobby>::SharedPtr m_playerJoinedLobbyPublisher;
    rclcpp::Publisher<turtle_checkers_interfaces::msg::PlayerLeftLobby>::SharedPtr m_playerLeftLobbyPublisher;
    rclcpp::Publisher<turtle_checkers_interfaces::msg::PlayerReadied>::SharedPtr m_playerReadiedPublisher;
    rclcpp::Publisher<turtle_checkers_interfaces::msg::UpdateLobbyOwner>::SharedPtr m_updateLobbyOwnerPublisher;
    rclcpp::Publisher<turtle_checkers_interfaces::msg::UpdateTimer>::SharedPtr m_updateTimerPublisher;

    rclcpp::Subscription<turtle_checkers_interfaces::msg::Forfit>::SharedPtr m_forfitSubscription;
    rclcpp::Subscription<turtle_checkers_interfaces::msg::KickPlayer>::SharedPtr m_kickPlayerSubscription;
    rclcpp::Subscription<turtle_checkers_interfaces::msg::OfferDraw>::SharedPtr m_offerDrawSubscription;
    rclcpp::Subscription<turtle_checkers_interfaces::msg::PlayerReady>::SharedPtr m_playerReadySubscription;
    rclcpp::Subscription<turtle_checkers_interfaces::msg::TimerChanged>::SharedPtr m_timerChangedSubscription;

    std::string m_lobbyName;
    std::string m_lobbyId;
    std::string m_lobbyOwnerPlayerName;
    std::string m_blackPlayerName;
    bool m_blackPlayerReady;
    std::string m_redPlayerName;
    bool m_redPlayerReady;
    GameState m_gameState;
    std::chrono::seconds m_timer;
    std::chrono::seconds m_blackTimeRemaining;
    std::chrono::seconds m_redTimeRemaining;
    std::chrono::time_point<std::chrono::system_clock> m_startTurnTimestamp;

    std::mutex m_timerMutex;
    std::thread m_timerThread;
    bool m_timerThreadRunning;

    std::string m_playerOfferingDraw;

    MasterBoardPtr m_board;

    uint64_t m_publicKey;
    uint64_t m_privateKey;
    uint64_t m_blackPlayerPublicKey;
    uint64_t m_redPlayerPublicKey;
    std::unordered_map<TurtlePieceColor, uint64_t> m_playerPublicKeysByColor;

    uint64_t m_lobbyPasswordHash;
};

typedef std::shared_ptr<CheckersGameLobby> CheckersGameLobbyPtr;