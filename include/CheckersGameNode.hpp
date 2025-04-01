#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"

#include "turtle_checkers_interfaces/srv/connect_to_game.hpp"
#include "turtle_checkers_interfaces/srv/request_piece_move.hpp"
#include "turtle_checkers_interfaces/srv/request_reachable_tiles.hpp"
#include "turtle_checkers_interfaces/msg/declare_winner.hpp"
#include "turtle_checkers_interfaces/msg/game_start.hpp"
#include "turtle_checkers_interfaces/msg/player_ready.hpp"
#include "turtle_checkers_interfaces/msg/update_board.hpp"

#include "CheckersGameLobby.hpp"

class GamePlayerNode : public rclcpp::Node
{
public:
    GamePlayerNode();

private:
    void connectToGameRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::ConnectToGame::Request> request,
                              std::shared_ptr<turtle_checkers_interfaces::srv::ConnectToGame::Response> response);
    void requestReachableTilesRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::RequestReachableTiles::Request> request,
                                      std::shared_ptr<turtle_checkers_interfaces::srv::RequestReachableTiles::Response> response);
    void requestPieceMoveRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::RequestPieceMove::Request> request,
                                 std::shared_ptr<turtle_checkers_interfaces::srv::RequestPieceMove::Response> response);
                                 
	void playerReadyCallback(const turtle_checkers_interfaces::msg::PlayerReady::SharedPtr message);

    rclcpp::Service<turtle_checkers_interfaces::srv::ConnectToGame>::SharedPtr m_connectToGameService;
    rclcpp::Service<turtle_checkers_interfaces::srv::RequestReachableTiles>::SharedPtr m_requestReachableTilesService;
    rclcpp::Service<turtle_checkers_interfaces::srv::RequestPieceMove>::SharedPtr m_requestPieceMoveService;

    rclcpp::Publisher<turtle_checkers_interfaces::msg::DeclareWinner>::SharedPtr m_declareWinnerPublisher;
    rclcpp::Publisher<turtle_checkers_interfaces::msg::GameStart>::SharedPtr m_gameStartPublisher;
    rclcpp::Publisher<turtle_checkers_interfaces::msg::UpdateBoard>::SharedPtr m_updateBoardPublisher;

    rclcpp::Subscription<turtle_checkers_interfaces::msg::PlayerReady>::SharedPtr m_playerReadySubscription;

    const std::string c_LOBBY_NAME_PREFIX = "TurtleCheckersLobby";
    size_t m_nextLobbyNumber;
    std::string m_nextLobbyName;
    std::unordered_map<std::string, CheckersGameLobbyPtr> m_checkersGameLobbies;
};