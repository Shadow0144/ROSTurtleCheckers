#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"

#include "turtle_checkers_interfaces/srv/connect_to_game.hpp"

#include "RSAKeyGenerator.hpp"
#include "CheckersGameLobby.hpp"

class CheckersGameMasterNode   
{
public:
    CheckersGameMasterNode();

    std::shared_ptr<rclcpp::Node> &getNodeHandle();

private:
    void connectToGameRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::ConnectToGame::Request> request,
                              std::shared_ptr<turtle_checkers_interfaces::srv::ConnectToGame::Response> response);

    rclcpp::Service<turtle_checkers_interfaces::srv::ConnectToGame>::SharedPtr m_connectToGameService;

    std::shared_ptr<rclcpp::Node> m_gameMasterNode;

    const std::string c_LOBBY_NAME_PREFIX = "TurtleCheckersLobby";
    size_t m_nextLobbyNumber;
    std::string m_nextLobbyName;
    std::unordered_map<std::string, CheckersGameLobbyPtr> m_checkersGameLobbies;

    long long m_privateKey;
    long long m_publicKey;
};