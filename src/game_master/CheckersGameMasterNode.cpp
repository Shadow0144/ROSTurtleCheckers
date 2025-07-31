#include "game_master/CheckersGameMasterNode.hpp"

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp" // For getting the styles directory

#include "turtle_checkers_interfaces/srv/connect_to_game_master.hpp"
#include "turtle_checkers_interfaces/srv/create_account.hpp"
#include "turtle_checkers_interfaces/srv/log_in_account.hpp"
#include "turtle_checkers_interfaces/srv/create_lobby.hpp"
#include "turtle_checkers_interfaces/srv/get_lobby_list.hpp"
#include "turtle_checkers_interfaces/srv/join_lobby.hpp"
#include "turtle_checkers_interfaces/msg/force_logout_account.hpp"
#include "turtle_checkers_interfaces/msg/leave_lobby.hpp"
#include "turtle_checkers_interfaces/msg/log_out_account.hpp"
#include "turtle_checkers_interfaces/msg/set_player_banned.hpp"

#include <memory>
#include <string>
#include <sstream>
#include <iomanip>
#include <fstream>

#include "shared/CheckersConsts.hpp"
#include "shared/Hasher.hpp"
#include "shared/RSAKeyGenerator.hpp"
#include "shared/TurtleLogger.hpp"
#include "game_master/CheckersGameLobby.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

const std::string authorizationKeyFile = "turtles_checkers_authorization_key.key";

CheckersGameMasterNode::CheckersGameMasterNode()
{
    // A game node creates a 2-player game for player nodes to publish their moves to
    // The game node publishes when it is ready for which player's next move, what the last move was, and when a winner is decided
    m_gameMasterNode = rclcpp::Node::make_shared("checkers_game_master_node");

    // Initialize the logger
    TurtleLogger::initialize("checkers_game_master_node");

    // Create the RSA key pair
    RSAKeyGenerator::generateRSAKeyPair(m_publicKey, m_privateKey);

    // Create/load the player database
    m_databaseHandler = std::make_unique<DatabaseHandler>(ament_index_cpp::get_package_share_directory("turtle_checkers") + "/db");

    // Create the publishers, subscriptions, and services

    m_forceLogoutAccountPublisher = m_gameMasterNode->create_publisher<turtle_checkers_interfaces::msg::ForceLogoutAccount>(
        "ForceLogoutAccount", 10);

    m_leaveLobbySubscription = m_gameMasterNode->create_subscription<turtle_checkers_interfaces::msg::LeaveLobby>(
        "LeaveLobby", 10, std::bind(&CheckersGameMasterNode::leaveLobbyCallback, this, std::placeholders::_1));
    m_logOutAccountSubscription = m_gameMasterNode->create_subscription<turtle_checkers_interfaces::msg::LogOutAccount>(
        "LogOutAccount", 10, std::bind(&CheckersGameMasterNode::logOutAccountCallback, this, std::placeholders::_1));
    m_setPlayerBannedSubscription = m_gameMasterNode->create_subscription<turtle_checkers_interfaces::msg::SetPlayerBanned>(
        "SetPlayerBanned", 10, std::bind(&CheckersGameMasterNode::setPlayerBannedCallback, this, std::placeholders::_1));

    m_connectToGameMasterService = m_gameMasterNode->create_service<turtle_checkers_interfaces::srv::ConnectToGameMaster>(
        "ConnectToGameMaster", std::bind(&CheckersGameMasterNode::connectToGameMasterRequest,
                                         this, std::placeholders::_1, std::placeholders::_2));
    m_createAccountService = m_gameMasterNode->create_service<turtle_checkers_interfaces::srv::CreateAccount>(
        "CreateAccount", std::bind(&CheckersGameMasterNode::createAccountRequest,
                                   this, std::placeholders::_1, std::placeholders::_2));
    m_logInAccountService = m_gameMasterNode->create_service<turtle_checkers_interfaces::srv::LogInAccount>(
        "LogInAccount", std::bind(&CheckersGameMasterNode::logInAccountRequest,
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

    // Get the authorization key from the file
    try
    {
        std::ifstream file(authorizationKeyFile);
        if (file)
        {
            file >> m_authorizationKey;
        }
        else
        {
            TurtleLogger::logError("Error opening authorization key file");
        }
    }
    catch (const std::exception &e)
    {
        TurtleLogger::logError(std::string("Authorization key file error: ") + e.what());
    }

    TurtleLogger::logInfo("Starting Turtles Checkers game node; now accepting players!");
}

void CheckersGameMasterNode::connectToGameMasterRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::ConnectToGameMaster::Request> request,
                                                        std::shared_ptr<turtle_checkers_interfaces::srv::ConnectToGameMaster::Response> response)
{
    (void)request; // NO LINT

    response->game_master_public_key = m_publicKey;
}

void CheckersGameMasterNode::createAccountRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::CreateAccount::Request> request,
                                                  std::shared_ptr<turtle_checkers_interfaces::srv::CreateAccount::Response> response)
{
    response->player_name = request->player_name;
    auto hashedPlayerPassword = RSAKeyGenerator::unencrypt(request->encrypted_hashed_player_password, m_privateKey, m_publicKey);
    response->created = m_databaseHandler->addPlayer(request->player_name, hashedPlayerPassword);
    response->error_msg = m_databaseHandler->getErrorMessage();

    response->checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::srv::CreateAccount::Response::SharedPtr>{}(response),
        m_publicKey, m_privateKey);

    m_playerPublicKeys[request->player_name] = request->player_public_key;
}

void CheckersGameMasterNode::logInAccountRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::LogInAccount::Request> request,
                                                 std::shared_ptr<turtle_checkers_interfaces::srv::LogInAccount::Response> response)
{
    response->player_name = request->player_name;

    // Checked if logged in already
    if (m_playerPublicKeys.find(request->player_name) != m_playerPublicKeys.end())
    {
        // Already logged in
        response->logged_in = false;
        response->error_msg = "Player already logged in";
    }
    else
    {
        // Check if the player has been banned
        if (m_databaseHandler->checkPlayerBanned(request->player_name))
        {
            // Already logged in
            response->logged_in = false;
            response->error_msg = "Player is banned";
        }
        else
        {
            auto hashedPlayerPassword = RSAKeyGenerator::unencrypt(request->encrypted_hashed_player_password, m_privateKey, m_publicKey);
            response->logged_in = m_databaseHandler->checkPasswordCorrect(request->player_name, hashedPlayerPassword);
            response->error_msg = m_databaseHandler->getErrorMessage();

            if (response->logged_in) // If logging in succeeded, add to the list
            {
                m_playerPublicKeys[request->player_name] = request->player_public_key;
            }
        }
    }

    response->checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::srv::LogInAccount::Response::SharedPtr>{}(response),
        m_publicKey, m_privateKey);
}

void CheckersGameMasterNode::createLobbyRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::CreateLobby::Request> request,
                                                std::shared_ptr<turtle_checkers_interfaces::srv::CreateLobby::Response> response)
{
    response->created = false;

    if (m_playerPublicKeys.find(request->player_name) != m_playerPublicKeys.end())
    {
        uint16_t attempts = 0u;
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
                                                                             m_publicKey,
                                                                             m_privateKey,
                                                                             lobbyName,
                                                                             lobbyId,
                                                                             unencryptedHashedLobbyPassword);
                m_checkersGameLobbies[fullName] = checkersGameLobby;
                checkersGameLobby->addPlayer(request->player_name, m_playerPublicKeys[request->player_name],
                                             static_cast<TurtlePieceColor>(request->desired_player_color));
                checkersGameLobby->setLobbyOwner(request->player_name);
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
            TurtleLogger::logWarn(lobbyAlreadyExistsError);
        }
    }
    else
    {
        std::string playerPublicKeyMissingError = "Player public key missing.";
        response->error_msg = playerPublicKeyMissingError;
        TurtleLogger::logWarn(playerPublicKeyMissingError);
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
    }

    response->checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::srv::GetLobbyList::Response::SharedPtr>{}(response),
        m_publicKey, m_privateKey);
}

void CheckersGameMasterNode::getStatisticsRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::GetStatistics::Request> request,
                                                  std::shared_ptr<turtle_checkers_interfaces::srv::GetStatistics::Response> response)
{
    const auto &playerStatistics = m_databaseHandler->getPlayerStatistics(request->player_name);

    response->player_name = request->player_name;
    for (const auto &match : playerStatistics.matchInfoList)
    {
        response->lobby_name_ids.push_back(match.lobbyNameId);
        response->black_player_names.push_back(match.blackPlayerName);
        response->red_player_names.push_back(match.redPlayerName);
        response->winners.push_back(match.winner);
    }
    response->matches_played = playerStatistics.matchesPlayed;
    response->matches_won = playerStatistics.matchesWon;
    response->matches_lost = playerStatistics.matchesLost;
    response->matches_drawed = playerStatistics.matchesDrawed;

    // No checksum required
}

void CheckersGameMasterNode::joinLobbyRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::JoinLobby::Request> request,
                                              std::shared_ptr<turtle_checkers_interfaces::srv::JoinLobby::Response> response)
{
    response->joined = false;

    if (m_playerPublicKeys.find(request->player_name) != m_playerPublicKeys.end())
    {
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
                                                     m_playerPublicKeys[request->player_name],
                                                     static_cast<TurtlePieceColor>(request->desired_player_color));
                        response->joined = true;
                        response->error_msg = "";
                        response->lobby_name = request->lobby_name;
                        response->lobby_id = request->lobby_id;
                        response->black_player_name = checkersGameLobby->getBlackPlayerName();
                        response->red_player_name = checkersGameLobby->getRedPlayerName();
                        response->black_player_ready = checkersGameLobby->getBlackPlayerReady();
                        response->red_player_ready = checkersGameLobby->getRedPlayerReady();
                        response->timer_seconds = checkersGameLobby->getTimerSeconds();
                    }
                    else
                    {
                        std::string playerAlreadyInLobbyError = "Player " + request->player_name + " already connected to this lobby.";
                        response->error_msg = playerAlreadyInLobbyError;
                        TurtleLogger::logWarn(playerAlreadyInLobbyError);
                    }
                }
                else
                {
                    std::string lobbyFullError = "Lobby is full.";
                    response->error_msg = lobbyFullError;
                    TurtleLogger::logWarn(lobbyFullError);
                }
            }
            else
            {
                std::string incorrectPasswordError = "Incorrect password.";
                response->error_msg = incorrectPasswordError;
                TurtleLogger::logInfo(incorrectPasswordError);
            }
        }
        else
        {
            std::string lobbyDoesNotExistError = "Lobby does not exist.";
            response->error_msg = lobbyDoesNotExistError;
            TurtleLogger::logWarn(lobbyDoesNotExistError);
        }
    }
    else
    {
        std::string playerPublicKeyMissingError = "Player public key missing.";
        response->error_msg = playerPublicKeyMissingError;
        TurtleLogger::logWarn(playerPublicKeyMissingError);
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

void CheckersGameMasterNode::logOutAccountCallback(const turtle_checkers_interfaces::msg::LogOutAccount::SharedPtr message)
{
    // TODO Confirm that the message has the correct key for the player
    if (m_playerPublicKeys.find(message->player_name) != m_playerPublicKeys.end())
    {
        m_playerPublicKeys.erase(message->player_name);
    }
}

void CheckersGameMasterNode::setPlayerBannedCallback(const turtle_checkers_interfaces::msg::SetPlayerBanned::SharedPtr message)
{
    // Check that the authorization key was loaded and matches
    if (m_authorizationKey == 0 || m_authorizationKey != message->authorization_key)
    {
        TurtleLogger::logWarn("Unauthorized attempt to access server");
        return;
    }

    m_databaseHandler->setPlayerBanned(message->player_name, message->banned);

    if (message->banned) // If the player was banned, force them to logout
    {
        auto forceLogoutAccountMessage = turtle_checkers_interfaces::msg::ForceLogoutAccount();
        forceLogoutAccountMessage.player_name = message->player_name;
        forceLogoutAccountMessage.checksum_sig = RSAKeyGenerator::createChecksumSignature(
            std::hash<turtle_checkers_interfaces::msg::ForceLogoutAccount>{}(forceLogoutAccountMessage),
            m_publicKey, m_privateKey);
        m_forceLogoutAccountPublisher->publish(forceLogoutAccountMessage);

        if (m_playerPublicKeys.find(message->player_name) != m_playerPublicKeys.end())
        {
            m_playerPublicKeys.erase(message->player_name);
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