#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"

#include "turtle_checkers_interfaces/srv/request_piece_move.hpp"
#include "turtle_checkers_interfaces/srv/request_reachable_tiles.hpp"
#include "turtle_checkers_interfaces/msg/declare_winner.hpp"
#include "turtle_checkers_interfaces/msg/game_start.hpp"
#include "turtle_checkers_interfaces/msg/player_ready.hpp"
#include "turtle_checkers_interfaces/msg/update_board.hpp"

#include "MasterBoard.hpp"

class CheckersGameLobby
{
public:
CheckersGameLobby(rclcpp::Node::SharedPtr &nodeHandle, const std::string &lobbyName);

    bool playerSlotAvailable() const;
    TurtlePieceColor addPlayer(const std::string &playerName);

    void setPlayerReady(const std::string &playerName);
    bool getAreAllPlayersReady() const;

    void setIsBlackTurn(bool isBlackTurn);
    bool getIsBlackTurn() const;
    void togglePlayerTurn();

private:
    void requestReachableTilesRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::RequestReachableTiles::Request> request,
                                      std::shared_ptr<turtle_checkers_interfaces::srv::RequestReachableTiles::Response> response);
    void requestPieceMoveRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::RequestPieceMove::Request> request,
                                 std::shared_ptr<turtle_checkers_interfaces::srv::RequestPieceMove::Response> response);

    void playerReadyCallback(const turtle_checkers_interfaces::msg::PlayerReady::SharedPtr message);

    bool isPieceValidForTurn(int requestedPieceTileIndex) const;

	rclcpp::Node::SharedPtr m_nodeHandle;

    rclcpp::Service<turtle_checkers_interfaces::srv::RequestReachableTiles>::SharedPtr m_requestReachableTilesService;
    rclcpp::Service<turtle_checkers_interfaces::srv::RequestPieceMove>::SharedPtr m_requestPieceMoveService;

    rclcpp::Publisher<turtle_checkers_interfaces::msg::DeclareWinner>::SharedPtr m_declareWinnerPublisher;
    rclcpp::Publisher<turtle_checkers_interfaces::msg::GameStart>::SharedPtr m_gameStartPublisher;
    rclcpp::Publisher<turtle_checkers_interfaces::msg::UpdateBoard>::SharedPtr m_updateBoardPublisher;

    rclcpp::Subscription<turtle_checkers_interfaces::msg::PlayerReady>::SharedPtr m_playerReadySubscription;

    std::string m_lobbyName;
    std::string m_blackPlayerName;
    bool m_blackPlayerReady;
    std::string m_redPlayerName;
    bool m_redPlayerReady;
    bool m_isBlackTurn;

    MasterBoardPtr m_board;
};

typedef std::shared_ptr<CheckersGameLobby> CheckersGameLobbyPtr;