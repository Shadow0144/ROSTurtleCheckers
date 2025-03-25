#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "turtle_checkers_interfaces/srv/request_reachable_tiles.hpp"

#include "CheckersGameLobby.hpp"

class GamePlayerNode : public rclcpp::Node
{
public:
    GamePlayerNode();

private:
    void requestReachableTilesRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::RequestReachableTiles::Request> request,
                                      std::shared_ptr<turtle_checkers_interfaces::srv::RequestReachableTiles::Response> response);

    rclcpp::Service<turtle_checkers_interfaces::srv::RequestReachableTiles>::SharedPtr m_requestReachableTilesService;

    CheckersGameLobbyPtr m_checkersGameLobby;
};