#include "game_master/CheckersGameMasterNode.hpp"

#include "rclcpp/rclcpp.hpp"

#include "turtle_checkers_interfaces/srv/connect_to_game_master.hpp"
#include "turtle_checkers_interfaces/srv/create_lobby.hpp"
#include "turtle_checkers_interfaces/srv/get_lobby_list.hpp"
#include "turtle_checkers_interfaces/srv/join_lobby.hpp"
#include "turtle_checkers_interfaces/msg/leave_lobby.hpp"

#include <memory>
#include <string>
#include <sstream>
#include <iomanip>

#include "shared/CheckersConsts.hpp"
#include "shared/RSAKeyGenerator.hpp"
#include "game_master/CheckersGameLobby.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

CheckersGameMasterNode::CheckersGameMasterNode()
{
    // Create the RSA key pair
    RSAKeyGenerator::generateRSAKeyPair(m_publicKey, m_privateKey);

    // A game node creates a 2-player game for player nodes to publish their moves to
    // The game node publishes when it is ready for which player's next move, what the last move was, and when a winner is decided
    m_gameMasterNode = rclcpp::Node::make_shared("checkers_game_master_node");

    m_connectToGameMasterService = m_gameMasterNode->create_service<turtle_checkers_interfaces::srv::ConnectToGameMaster>(
        "ConnectToGameMaster", std::bind(&CheckersGameMasterNode::connectToGameMasterRequest,
                                         this, std::placeholders::_1, std::placeholders::_2));
    m_createLobbyService = m_gameMasterNode->create_service<turtle_checkers_interfaces::srv::CreateLobby>(
        "CreateLobby", std::bind(&CheckersGameMasterNode::createLobbyRequest,
                                 this, std::placeholders::_1, std::placeholders::_2));
    m_getLobbyListService = m_gameMasterNode->create_service<turtle_checkers_interfaces::srv::GetLobbyList>(
        "GetLobbyList", std::bind(&CheckersGameMasterNode::getLobbyListRequest,
                                  this, std::placeholders::_1, std::placeholders::_2));
    m_joinLobbyService = m_gameMasterNode->create_service<turtle_checkers_interfaces::srv::JoinLobby>(
        "JoinLobby", std::bind(&CheckersGameMasterNode::joinLobbyRequest,
                               this, std::placeholders::_1, std::placeholders::_2));

    m_leaveLobbySubscription = m_gameMasterNode->create_subscription<turtle_checkers_interfaces::msg::LeaveLobby>(
        "LeaveLobby", 10, std::bind(&CheckersGameMasterNode::leaveLobbyCallback, this, std::placeholders::_1));

    RCLCPP_INFO(m_gameMasterNode->get_logger(), "Starting Turtles Checkers game node; now accepting players!");
}

void CheckersGameMasterNode::connectToGameMasterRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::ConnectToGameMaster::Request> request,
                                                        std::shared_ptr<turtle_checkers_interfaces::srv::ConnectToGameMaster::Response> response)
{
    (void)request; // NO LINT

    response->game_master_public_key = m_publicKey;
}

void CheckersGameMasterNode::createLobbyRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::CreateLobby::Request> request,
                                                std::shared_ptr<turtle_checkers_interfaces::srv::CreateLobby::Response> response)
{
    response->created = false;
    uint16_t attempts = 0u;

    // TODO: Check player name and lobby name

    do
    {
        std::string lobbyName = request->lobby_name;
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(4) << std::to_string(m_nextLobbyId); // Add leading 0s
        std::string lobbyId = ss.str();
        m_nextLobbyId = (m_nextLobbyId + 1u) % MAX_LOBBY_LIMIT; // Increment the ID
        attempts++;
        std::string fullName = lobbyName + "#" + lobbyId;
        uint32_t encryptedHashedLobbyPassword = request->encrypted_hashed_lobby_password; // Encrypted with own public key
        uint32_t unencryptedHashedLobbyPassword = RSAKeyGenerator::unencrypt(encryptedHashedLobbyPassword, m_privateKey, m_publicKey);
        if (m_checkersGameLobbies.find(fullName) == m_checkersGameLobbies.end())
        {
            auto checkersGameLobby = std::make_shared<CheckersGameLobby>(m_gameMasterNode,
                                                                         m_publicKey, m_privateKey,
                                                                         lobbyName,
                                                                         lobbyId,
                                                                         unencryptedHashedLobbyPassword);
            m_checkersGameLobbies[fullName] = checkersGameLobby;
            checkersGameLobby->addPlayer(request->player_name,
                                         request->player_public_key,
                                         static_cast<TurtlePieceColor>(request->desired_player_color));
            response->created = true;
            response->error_msg = "";
            response->lobby_name = lobbyName;
            response->lobby_id = lobbyId;
            response->black_player_name = checkersGameLobby->getBlackPlayerName();
            response->red_player_name = checkersGameLobby->getRedPlayerName();
            response->black_player_ready = checkersGameLobby->getBlackPlayerReady();
            response->red_player_ready = checkersGameLobby->getRedPlayerReady();
        }
    } while (!response->created && attempts <= MAX_LOBBY_LIMIT);

    if (!response->created)
    {
        std::string lobbyAlreadyExistsError = "Lobby already exists.";
        response->error_msg = lobbyAlreadyExistsError;
        RCLCPP_WARN(m_gameMasterNode->get_logger(), lobbyAlreadyExistsError);
    }

    response->checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::srv::CreateLobby::Response::SharedPtr>{}(response),
        m_publicKey, m_privateKey);
}

void CheckersGameMasterNode::getLobbyListRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::GetLobbyList::Request> request,
                                                 std::shared_ptr<turtle_checkers_interfaces::srv::GetLobbyList::Response> response)
{
    (void)request; // NO LINT

    for (const auto &pair : m_checkersGameLobbies)
    {
        response->lobby_names.push_back(pair.second->getLobbyName());
        response->lobby_ids.push_back(pair.second->getLobbyId());
        response->has_passwords.push_back(pair.second->hasPassword());
        response->joined_black_player_names.push_back(pair.second->getBlackPlayerName());
        response->joined_red_player_names.push_back(pair.second->getRedPlayerName());
        response->checksum_sig = RSAKeyGenerator::createChecksumSignature(
            std::hash<turtle_checkers_interfaces::srv::GetLobbyList::Response::SharedPtr>{}(response),
            m_publicKey, m_privateKey);
    }
}

void CheckersGameMasterNode::joinLobbyRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::JoinLobby::Request> request,
                                              std::shared_ptr<turtle_checkers_interfaces::srv::JoinLobby::Response> response)
{
    response->joined = false;

    // TODO: Check player name

    auto lobbyName = request->lobby_name + "#" + request->lobby_id;
    if (m_checkersGameLobbies.find(lobbyName) != m_checkersGameLobbies.end())
    {
        auto &checkersGameLobby = m_checkersGameLobbies[lobbyName];

        uint32_t encryptedHashedLobbyPassword = request->encrypted_hashed_lobby_password; // Encrypted with own public key
        uint32_t unencryptedHashedLobbyPassword = RSAKeyGenerator::unencrypt(encryptedHashedLobbyPassword, m_privateKey, m_publicKey);
        if (checkersGameLobby->passwordMatches(unencryptedHashedLobbyPassword))
        {
            if (checkersGameLobby->isPlayerSlotAvailable())
            {
                if (!checkersGameLobby->containsPlayer(request->player_name))
                {
                    checkersGameLobby->addPlayer(request->player_name,
                                                 request->player_public_key,
                                                 static_cast<TurtlePieceColor>(request->desired_player_color));
                    response->joined = true;
                    response->error_msg = "";
                    response->lobby_name = request->lobby_name;
                    response->lobby_id = request->lobby_id;
                    response->black_player_name = checkersGameLobby->getBlackPlayerName();
                    response->red_player_name = checkersGameLobby->getRedPlayerName();
                    response->black_player_ready = checkersGameLobby->getBlackPlayerReady();
                    response->red_player_ready = checkersGameLobby->getRedPlayerReady();
                }
                else
                {
                    std::string playerAlreadyInLobbyError = "Player " + request->player_name + " already connected to this lobby.";
                    response->error_msg = playerAlreadyInLobbyError;
                    RCLCPP_WARN(m_gameMasterNode->get_logger(), playerAlreadyInLobbyError);
                }
            }
            else
            {
                std::string lobbyFullError = "Lobby is full.";
                response->error_msg = lobbyFullError;
                RCLCPP_WARN(m_gameMasterNode->get_logger(), lobbyFullError);
            }
        }
        else
        {
            std::string incorrectPasswordError = "Incorrect password.";
            response->error_msg = incorrectPasswordError;
            RCLCPP_INFO(m_gameMasterNode->get_logger(), incorrectPasswordError);
        }
    }
    else
    {
        std::string lobbyDoesNotExistError = "Lobby does not exist.";
        response->error_msg = lobbyDoesNotExistError;
        RCLCPP_WARN(m_gameMasterNode->get_logger(), lobbyDoesNotExistError);
    }

    response->checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::srv::JoinLobby::Response::SharedPtr>{}(response),
        m_publicKey, m_privateKey);
}

void CheckersGameMasterNode::leaveLobbyCallback(const turtle_checkers_interfaces::msg::LeaveLobby::SharedPtr message)
{
    // TODO Confirm that the message has the correct key for the player
    auto lobbyName = message->lobby_name + "#" + message->lobby_id;
    if (m_checkersGameLobbies.find(lobbyName) != m_checkersGameLobbies.end())
    {
        auto &checkersGameLobby = m_checkersGameLobbies[lobbyName];
        checkersGameLobby->removePlayer(message->player_name);

        if (checkersGameLobby->isLobbyEmpty())
        {
            // If the lobby is empty, close it
            m_checkersGameLobbies.erase(lobbyName);
        }
    }
}

std::shared_ptr<rclcpp::Node> &CheckersGameMasterNode::getNodeHandle()
{
    return m_gameMasterNode;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto gameMasterNode = std::make_shared<CheckersGameMasterNode>();
    rclcpp::spin(gameMasterNode->getNodeHandle());
    rclcpp::shutdown();
    return 0;
}