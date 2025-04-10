#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"

#include "turtle_checkers_interfaces/srv/connect_to_game_master.hpp"
#include "turtle_checkers_interfaces/srv/create_lobby.hpp"
#include "turtle_checkers_interfaces/srv/get_lobby_list.hpp"
#include "turtle_checkers_interfaces/srv/join_lobby.hpp"
#include "turtle_checkers_interfaces/msg/leave_lobby.hpp"

#include "shared/RSAKeyGenerator.hpp"
#include "game_master/CheckersGameLobby.hpp"

class CheckersGameMasterNode
{
public:
    CheckersGameMasterNode();

    std::shared_ptr<rclcpp::Node> &getNodeHandle();

private:
    void connectToGameMasterRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::ConnectToGameMaster::Request> request,
                                    std::shared_ptr<turtle_checkers_interfaces::srv::ConnectToGameMaster::Response> response);
    void joinLobbyRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::JoinLobby::Request> request,
                          std::shared_ptr<turtle_checkers_interfaces::srv::JoinLobby::Response> response);
    void createLobbyRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::CreateLobby::Request> request,
                            std::shared_ptr<turtle_checkers_interfaces::srv::CreateLobby::Response> response);
    void getLobbyListRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::GetLobbyList::Request> request,
                             std::shared_ptr<turtle_checkers_interfaces::srv::GetLobbyList::Response> response);

    void leaveLobbyCallback(const turtle_checkers_interfaces::msg::LeaveLobby::SharedPtr message);

    rclcpp::Service<turtle_checkers_interfaces::srv::ConnectToGameMaster>::SharedPtr m_connectToGameMasterService;
    rclcpp::Service<turtle_checkers_interfaces::srv::CreateLobby>::SharedPtr m_createLobbyService;
    rclcpp::Service<turtle_checkers_interfaces::srv::GetLobbyList>::SharedPtr m_getLobbyListService;
    rclcpp::Service<turtle_checkers_interfaces::srv::JoinLobby>::SharedPtr m_joinLobbyService;
    
    rclcpp::Subscription<turtle_checkers_interfaces::msg::LeaveLobby>::SharedPtr m_leaveLobbySubscription;

    std::shared_ptr<rclcpp::Node> m_gameMasterNode;

    std::unordered_map<std::string, CheckersGameLobbyPtr> m_checkersGameLobbies;

    long long m_privateKey;
    long long m_publicKey;
};