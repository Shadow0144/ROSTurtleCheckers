#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"

#include "turtle_checkers_interfaces/srv/request_piece_move.hpp"
#include "turtle_checkers_interfaces/srv/request_reachable_tiles.hpp"
#include "turtle_checkers_interfaces/srv/request_board_state.hpp"
#include "turtle_checkers_interfaces/msg/declare_winner.hpp"
#include "turtle_checkers_interfaces/msg/game_start.hpp"
#include "turtle_checkers_interfaces/msg/player_joined_lobby.hpp"
#include "turtle_checkers_interfaces/msg/player_left_lobby.hpp"
#include "turtle_checkers_interfaces/msg/player_ready.hpp"
#include "turtle_checkers_interfaces/msg/update_board.hpp"

#include "game_master/MasterBoard.hpp"

class CheckersGameLobby
{
public:
    CheckersGameLobby(rclcpp::Node::SharedPtr &nodeHandle,
                      const std::string &lobbyName,
                      const std::string &lobbyId);

    const std::string &getLobbyName() const;
    const std::string &getLobbyId() const;

    bool isLobbyEmpty() const;
    bool isPlayerSlotAvailable() const;
    bool containsPlayer(const std::string &playerName) const;
    TurtlePieceColor addPlayer(const std::string &playerName, TurtlePieceColor desiredColor);
    void removePlayer(const std::string &playerName);

    const std::string &getBlackPlayerName() const;
    const std::string &getRedPlayerName() const;

    bool getBlackPlayerReady() const;
    bool getRedPlayerReady() const;
    void setPlayerReady(const std::string &playerName, bool ready);
    bool getAreAllPlayersReady() const;

    void setIsBlackTurn(bool isBlackTurn);
    bool getIsBlackTurn() const;
    void togglePlayerTurn();

private:
    void requestReachableTilesRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::RequestReachableTiles::Request> request,
                                      std::shared_ptr<turtle_checkers_interfaces::srv::RequestReachableTiles::Response> response);
    void requestPieceMoveRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::RequestPieceMove::Request> request,
                                 std::shared_ptr<turtle_checkers_interfaces::srv::RequestPieceMove::Response> response);
    void requestBoardStateRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::RequestBoardState::Request> request,
                                  std::shared_ptr<turtle_checkers_interfaces::srv::RequestBoardState::Response> response);

    void playerReadyCallback(const turtle_checkers_interfaces::msg::PlayerReady::SharedPtr message);

    bool isPieceValidForTurn(int requestedPieceTileIndex) const;

    rclcpp::Node::SharedPtr m_nodeHandle;

    rclcpp::Service<turtle_checkers_interfaces::srv::RequestReachableTiles>::SharedPtr m_requestReachableTilesService;
    rclcpp::Service<turtle_checkers_interfaces::srv::RequestPieceMove>::SharedPtr m_requestPieceMoveService;
    rclcpp::Service<turtle_checkers_interfaces::srv::RequestBoardState>::SharedPtr m_requestBoardStateService;

    rclcpp::Publisher<turtle_checkers_interfaces::msg::DeclareWinner>::SharedPtr m_declareWinnerPublisher;
    rclcpp::Publisher<turtle_checkers_interfaces::msg::GameStart>::SharedPtr m_gameStartPublisher;
    rclcpp::Publisher<turtle_checkers_interfaces::msg::UpdateBoard>::SharedPtr m_updateBoardPublisher;
    rclcpp::Publisher<turtle_checkers_interfaces::msg::PlayerJoinedLobby>::SharedPtr m_playerJoinedLobbyPublisher;
    rclcpp::Publisher<turtle_checkers_interfaces::msg::PlayerLeftLobby>::SharedPtr m_playerLeftLobbyPublisher;

    rclcpp::Subscription<turtle_checkers_interfaces::msg::PlayerReady>::SharedPtr m_playerReadySubscription;

    std::string m_lobbyName;
    std::string m_lobbyId;
    std::string m_blackPlayerName;
    bool m_blackPlayerReady;
    std::string m_redPlayerName;
    bool m_redPlayerReady;
    bool m_isBlackTurn;

    MasterBoardPtr m_board;
};

typedef std::shared_ptr<CheckersGameLobby> CheckersGameLobbyPtr;