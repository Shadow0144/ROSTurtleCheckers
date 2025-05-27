#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"

#include "turtle_checkers_interfaces/srv/connect_to_game_master.hpp"
#include "turtle_checkers_interfaces/srv/create_account.hpp"
#include "turtle_checkers_interfaces/srv/log_in_account.hpp"
#include "turtle_checkers_interfaces/srv/create_lobby.hpp"
#include "turtle_checkers_interfaces/srv/get_lobby_list.hpp"
#include "turtle_checkers_interfaces/srv/join_lobby.hpp"
#include "turtle_checkers_interfaces/msg/leave_lobby.hpp"
#include "turtle_checkers_interfaces/msg/log_out_account.hpp"

#include "shared/RSAKeyGenerator.hpp"
#include "game_master/DatabaseHandler.hpp"
#include "game_master/CheckersGameLobby.hpp"

class CheckersGameMasterNode
{
public:
    CheckersGameMasterNode();

    std::shared_ptr<rclcpp::Node> &getNodeHandle();

private:
    void connectToGameMasterRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::ConnectToGameMaster::Request> request,
                                    std::shared_ptr<turtle_checkers_interfaces::srv::ConnectToGameMaster::Response> response);
    void createAccountRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::CreateAccount::Request> request,
                              std::shared_ptr<turtle_checkers_interfaces::srv::CreateAccount::Response> response);
    void logInAccountRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::LogInAccount::Request> request,
                             std::shared_ptr<turtle_checkers_interfaces::srv::LogInAccount::Response> response);
    void joinLobbyRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::JoinLobby::Request> request,
                          std::shared_ptr<turtle_checkers_interfaces::srv::JoinLobby::Response> response);
    void createLobbyRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::CreateLobby::Request> request,
                            std::shared_ptr<turtle_checkers_interfaces::srv::CreateLobby::Response> response);
    void getLobbyListRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::GetLobbyList::Request> request,
                             std::shared_ptr<turtle_checkers_interfaces::srv::GetLobbyList::Response> response);

    void leaveLobbyCallback(const turtle_checkers_interfaces::msg::LeaveLobby::SharedPtr message);
    void logOutAccountCallback(const turtle_checkers_interfaces::msg::LogOutAccount::SharedPtr message);

    rclcpp::Service<turtle_checkers_interfaces::srv::ConnectToGameMaster>::SharedPtr m_connectToGameMasterService;
    rclcpp::Service<turtle_checkers_interfaces::srv::CreateAccount>::SharedPtr m_createAccountService;
    rclcpp::Service<turtle_checkers_interfaces::srv::LogInAccount>::SharedPtr m_logInAccountService;
    rclcpp::Service<turtle_checkers_interfaces::srv::CreateLobby>::SharedPtr m_createLobbyService;
    rclcpp::Service<turtle_checkers_interfaces::srv::GetLobbyList>::SharedPtr m_getLobbyListService;
    rclcpp::Service<turtle_checkers_interfaces::srv::JoinLobby>::SharedPtr m_joinLobbyService;

    rclcpp::Subscription<turtle_checkers_interfaces::msg::LeaveLobby>::SharedPtr m_leaveLobbySubscription;
    rclcpp::Subscription<turtle_checkers_interfaces::msg::LogOutAccount>::SharedPtr m_logOutAccountSubscription;

    std::shared_ptr<rclcpp::Node> m_gameMasterNode;
    
    DatabaseHandlerUniPtr m_databaseHandler;

    std::unordered_map<std::string, uint64_t> m_playerPublicKeys;
    std::unordered_map<std::string, CheckersGameLobbyPtr> m_checkersGameLobbies;

    uint16_t MAX_LOBBY_LIMIT = 1000; // Max ID is 1 less, they then loop around
    uint16_t m_nextLobbyId;          // [0 - MAX_LOBBY_LIMIT)

    uint64_t m_publicKey;
    uint64_t m_privateKey;
};