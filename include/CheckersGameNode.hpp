#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "turtle_checkers_interfaces/srv/connect_to_game.hpp"
#include "turtle_checkers_interfaces/srv/request_reachable_tiles.hpp"
#include "turtle_checkers_interfaces/srv/request_piece_move.hpp"
#include "turtle_checkers_interfaces/msg/declare_winner.hpp"
#include "turtle_checkers_interfaces/msg/update_board.hpp"
#include "turtle_checkers_interfaces/msg/update_game_state.hpp"

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

    rclcpp::Service<turtle_checkers_interfaces::srv::ConnectToGame>::SharedPtr m_connectToGameService;
    rclcpp::Service<turtle_checkers_interfaces::srv::RequestReachableTiles>::SharedPtr m_requestReachableTilesService;
    rclcpp::Service<turtle_checkers_interfaces::srv::RequestPieceMove>::SharedPtr m_requestPieceMoveService;

    rclcpp::Publisher<turtle_checkers_interfaces::msg::DeclareWinner>::SharedPtr m_declareWinnerPublisher;
    rclcpp::Publisher<turtle_checkers_interfaces::msg::UpdateGameState>::SharedPtr m_updateGameStatePublisher;
    rclcpp::Publisher<turtle_checkers_interfaces::msg::UpdateBoard>::SharedPtr m_updateBoardPublisher;

    CheckersGameLobbyPtr m_checkersGameLobby;
};