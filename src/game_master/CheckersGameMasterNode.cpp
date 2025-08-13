#include "game_master/CheckersGameMasterNode.hpp"

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp" // For getting the styles directory

#include "turtle_checkers_interfaces/msg/client_heartbeat.hpp"
#include "turtle_checkers_interfaces/msg/leave_lobby.hpp"
#include "turtle_checkers_interfaces/msg/log_out_account.hpp"
#include "turtle_checkers_interfaces/msg/player_banned.hpp"
#include "turtle_checkers_interfaces/msg/player_logged_out.hpp"
#include "turtle_checkers_interfaces/msg/report_player.hpp"
#include "turtle_checkers_interfaces/msg/server_heartbeat.hpp"
#include "turtle_checkers_interfaces/msg/set_player_banned.hpp"
#include "turtle_checkers_interfaces/srv/connect_to_game_master.hpp"
#include "turtle_checkers_interfaces/srv/create_account.hpp"
#include "turtle_checkers_interfaces/srv/log_in_account.hpp"
#include "turtle_checkers_interfaces/srv/create_lobby.hpp"
#include "turtle_checkers_interfaces/srv/get_lobby_list.hpp"
#include "turtle_checkers_interfaces/srv/get_statistics.hpp"
#include "turtle_checkers_interfaces/srv/join_lobby.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <regex>
#include <unordered_map>
#include <thread>

#include "shared/CheckersConsts.hpp"
#include "shared/Hasher.hpp"
#include "shared/RSAKeyGenerator.hpp"
#include "shared/TurtleLogger.hpp"
#include "game_master/CheckersGameLobby.hpp"
#include "game_master/DatabaseHandler.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

const std::string authorizationKeyFile = "turtle_checkers_authorization_key.key";
const std::string reportEmailFile = "report_email.config";

bool isValidEmail(const std::string &email)
{
    const std::regex pattern(R"(^[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}$)");
    return std::regex_match(email, pattern);
}

bool isValidUnsignedInt(const std::string &number)
{
    static const std::regex pattern(R"(^\d+$)");
    return std::regex_match(number, pattern);
}

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
    m_databaseHandler = std::make_shared<DatabaseHandler>(ament_index_cpp::get_package_share_directory("turtle_checkers") + "/db");

    // Create the publishers, subscriptions, and services

    m_playerBannedPublisher = m_gameMasterNode->create_publisher<turtle_checkers_interfaces::msg::PlayerBanned>(
        "PlayerBanned", 10);
    m_playerLoggedOutPublisher = m_gameMasterNode->create_publisher<turtle_checkers_interfaces::msg::PlayerLoggedOut>(
        "PlayerLoggedOut", 10);
    m_serverHeartbeatPublisher = m_gameMasterNode->create_publisher<turtle_checkers_interfaces::msg::ServerHeartbeat>(
        "ServerHeartbeat", 10);

    m_clientHeartbeatSubscription = m_gameMasterNode->create_subscription<turtle_checkers_interfaces::msg::ClientHeartbeat>(
        "ClientHeartbeat", 10, std::bind(&CheckersGameMasterNode::clientHeartbeatCallback, this, std::placeholders::_1));
    m_leaveLobbySubscription = m_gameMasterNode->create_subscription<turtle_checkers_interfaces::msg::LeaveLobby>(
        "LeaveLobby", 10, std::bind(&CheckersGameMasterNode::leaveLobbyCallback, this, std::placeholders::_1));
    m_logOutAccountSubscription = m_gameMasterNode->create_subscription<turtle_checkers_interfaces::msg::LogOutAccount>(
        "LogOutAccount", 10, std::bind(&CheckersGameMasterNode::logOutAccountCallback, this, std::placeholders::_1));
    m_reportPlayerSubscription = m_gameMasterNode->create_subscription<turtle_checkers_interfaces::msg::ReportPlayer>(
        "ReportPlayer", 10, std::bind(&CheckersGameMasterNode::reportPlayerCallback, this, std::placeholders::_1));
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
    m_getStatisticsService = m_gameMasterNode->create_service<turtle_checkers_interfaces::srv::GetStatistics>(
        "GetStatistics", std::bind(&CheckersGameMasterNode::getStatisticsRequest,
                                   this, std::placeholders::_1, std::placeholders::_2));
    m_joinLobbyService = m_gameMasterNode->create_service<turtle_checkers_interfaces::srv::JoinLobby>(
        "JoinLobby", std::bind(&CheckersGameMasterNode::joinLobbyRequest,
                               this, std::placeholders::_1, std::placeholders::_2));

    // Get the authorization key from the file
    try
    {
        std::ifstream file(ament_index_cpp::get_package_share_directory("turtle_checkers") + "/" + authorizationKeyFile);
        if (file)
        {
            std::string key;
            file >> key;
            if (isValidUnsignedInt(key))
            {
                m_authorizationKey = std::stoull(key);
            }
            else
            {
                TurtleLogger::logError("Authorization key must be an unsigned 64-bit integer");
            }
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

    // Get the report email address from the file
    try
    {
        std::ifstream file(ament_index_cpp::get_package_share_directory("turtle_checkers") + "/" + reportEmailFile);
        if (file)
        {
            file >> m_reportEmailAddress;
            if (!isValidEmail(m_reportEmailAddress))
            {
                m_reportEmailAddress.clear();
            }
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

    // Start up the heartbeat thread
    m_heartbeatThreadRunning = true;
    m_heartbeatThread = std::thread(&CheckersGameMasterNode::handleHeartbeats, this);

    TurtleLogger::logInfo("Starting Turtles Checkers game node; now accepting players!");
}

CheckersGameMasterNode::~CheckersGameMasterNode()
{
    m_heartbeatThreadRunning = false;
    if (m_heartbeatThread.joinable())
    {
        m_heartbeatThread.join();
    }
}

void CheckersGameMasterNode::clientHeartbeatCallback(const turtle_checkers_interfaces::msg::ClientHeartbeat::SharedPtr message)
{
    uint64_t playerPublicKey = 0u;
    {
        std::lock_guard<std::mutex> lock(m_playerKeysMutex);
        if (m_playerPublicKeys.find(message->player_name) != m_playerPublicKeys.end())
        {
            playerPublicKey = m_playerPublicKeys[message->player_name];
        }
    }

    if (playerPublicKey == 0u ||
        !RSAKeyGenerator::checksumSignatureMatches(
            std::hash<turtle_checkers_interfaces::msg::ClientHeartbeat>{}(*message),
            playerPublicKey, message->checksum_sig))
    {
        std::cerr << "Checksum failed" << std::endl;
        return; // Checksum did not match with the public key
    }

    // The timestamp in the message is only so the checksum is unique; use the local time instead of trusting the player's clock
    std::lock_guard<std::mutex> lock(m_heartbeatMutex);
    m_playerHeartbeatTimestamps[message->player_name] = std::chrono::system_clock::now();
}

void CheckersGameMasterNode::leaveLobbyCallback(const turtle_checkers_interfaces::msg::LeaveLobby::SharedPtr message)
{
    auto lobbyName = message->lobby_name + "#" + message->lobby_id;
    if (m_checkersGameLobbies.find(lobbyName) != m_checkersGameLobbies.end())
    {
        uint64_t playerPublicKey = 0u;
        {
            std::lock_guard<std::mutex> lock(m_playerKeysMutex);
            if (m_playerPublicKeys.find(message->player_name) != m_playerPublicKeys.end())
            {
                playerPublicKey = m_playerPublicKeys[message->player_name];
            }
        }

        if (playerPublicKey == 0u ||
            !RSAKeyGenerator::checksumSignatureMatches(
                std::hash<turtle_checkers_interfaces::msg::LeaveLobby>{}(*message),
                playerPublicKey, message->checksum_sig))
        {
            std::cerr << "Checksum failed" << std::endl;
            return; // Checksum did not match with the public key
        }

        auto &checkersGameLobby = m_checkersGameLobbies[lobbyName];
        checkersGameLobby->removePlayer(message->player_name, false);

        if (checkersGameLobby->isLobbyEmpty())
        {
            // If the lobby is empty, close it
            m_checkersGameLobbies.erase(lobbyName);
        }
    }
}

void CheckersGameMasterNode::logOutAccountCallback(const turtle_checkers_interfaces::msg::LogOutAccount::SharedPtr message)
{
    std::lock_guard<std::mutex> lock(m_playerKeysMutex);
    if (m_playerPublicKeys.find(message->player_name) != m_playerPublicKeys.end())
    {
        uint64_t playerPublicKey = m_playerPublicKeys[message->player_name];
        if (!RSAKeyGenerator::checksumSignatureMatches(
                std::hash<turtle_checkers_interfaces::msg::LogOutAccount>{}(*message),
                playerPublicKey, message->checksum_sig))
        {
            std::cerr << "Checksum failed" << std::endl;
            return; // Checksum did not match with the public key
        }

        // Log them out
        m_playerPublicKeys.erase(message->player_name);
        m_playerHeartbeatTimestamps.erase(message->player_name);

        TurtleLogger::logInfo("Player " + message->player_name + " has logged out");
    }
}

void CheckersGameMasterNode::reportPlayerCallback(const turtle_checkers_interfaces::msg::ReportPlayer::SharedPtr message)
{
    if (m_reportEmailAddress.empty())
    {
        std::cerr << "Report email address not configured in report_email.config" << std::endl;
        return; // No where to send a report to, exit
    }

    uint64_t playerPublicKey = 0u;
    {
        std::lock_guard<std::mutex> lock(m_playerKeysMutex);
        if (m_playerPublicKeys.find(message->reporting_player_name) != m_playerPublicKeys.end())
        {
            playerPublicKey = m_playerPublicKeys[message->reporting_player_name];
        }
    }

    if (playerPublicKey == 0u ||
        !RSAKeyGenerator::checksumSignatureMatches(
            std::hash<turtle_checkers_interfaces::msg::ReportPlayer>{}(*message),
            playerPublicKey, message->checksum_sig))
    {
        std::cerr << "Checksum failed" << std::endl;
        return; // Checksum did not match with the public key
    }

    const auto recipient = m_reportEmailAddress;
    const auto subject = std::string("Turtles Checkers Player Reported");
    const auto body = std::string("Reporting player: " + message->reporting_player_name + "\n" +
                                  "Reported player: " + message->reported_player_name + "\n" +
                                  "\nChat log:\n" + message->chat_messages);

    // Build the Linux command string
    std::string command = "echo \"" + body + "\" | mail -s \"" + subject + "\" " + recipient;

    // Call the system mail command
    int result = std::system(command.c_str());

    if (result != 0)
    {
        std::cerr << "Failed to send report email" << std::endl;
    }
}

void CheckersGameMasterNode::setPlayerBannedCallback(const turtle_checkers_interfaces::msg::SetPlayerBanned::SharedPtr message)
{
    // Check that the authorization key was loaded and matches
    if (m_authorizationKey == 0u || m_authorizationKey != message->authorization_key)
    {
        TurtleLogger::logWarn("Unauthorized attempt to access server");
        return;
    }

    m_databaseHandler->setPlayerBanned(message->player_name, message->banned);

    if (message->banned) // If the player was banned, force them to logout
    {
        auto playerBannedMessage = turtle_checkers_interfaces::msg::PlayerBanned();
        playerBannedMessage.player_name = message->player_name;
        playerBannedMessage.checksum_sig = RSAKeyGenerator::createChecksumSignature(
            std::hash<turtle_checkers_interfaces::msg::PlayerBanned>{}(playerBannedMessage),
            m_publicKey, m_privateKey);
        m_playerBannedPublisher->publish(playerBannedMessage);

        {
            std::lock_guard<std::mutex> lock(m_playerKeysMutex);
            if (m_playerPublicKeys.find(message->player_name) != m_playerPublicKeys.end())
            {
                m_playerPublicKeys.erase(message->player_name);
                m_playerHeartbeatTimestamps.erase(message->player_name);
            }
        }
    }
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

    std::lock_guard<std::mutex> lock(m_playerKeysMutex);
    m_playerPublicKeys[request->player_name] = request->player_public_key;
}

void CheckersGameMasterNode::logInAccountRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::LogInAccount::Request> request,
                                                 std::shared_ptr<turtle_checkers_interfaces::srv::LogInAccount::Response> response)
{
    response->player_name = request->player_name;

    uint64_t playerPublicKey = 0u;
    {
        std::lock_guard<std::mutex> lock(m_playerKeysMutex);
        if (m_playerPublicKeys.find(request->player_name) != m_playerPublicKeys.end())
        {
            playerPublicKey = m_playerPublicKeys[request->player_name];
        }
    }

    // Checked if logged in already
    if (playerPublicKey != 0u)
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
                std::lock_guard<std::mutex> lock(m_playerKeysMutex);
                m_playerPublicKeys[request->player_name] = request->player_public_key;
                m_playerHeartbeatTimestamps[request->player_name] = std::chrono::system_clock::now();
            }
        }
    }

    response->checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::srv::LogInAccount::Response::SharedPtr>{}(response),
        m_publicKey, m_privateKey);

    TurtleLogger::logInfo("Player " + request->player_name + " has logged in");
}

void CheckersGameMasterNode::createLobbyRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::CreateLobby::Request> request,
                                                std::shared_ptr<turtle_checkers_interfaces::srv::CreateLobby::Response> response)
{
    response->created = false;

    uint64_t playerPublicKey = 0u;
    {
        std::lock_guard<std::mutex> lock(m_playerKeysMutex);
        if (m_playerPublicKeys.find(request->player_name) != m_playerPublicKeys.end())
        {
            playerPublicKey = m_playerPublicKeys[request->player_name];
        }
    }

    if (playerPublicKey != 0u)
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
                                                                             unencryptedHashedLobbyPassword,
                                                                             m_databaseHandler);
                m_checkersGameLobbies[fullName] = checkersGameLobby;
                checkersGameLobby->addPlayer(request->player_name, playerPublicKey,
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
    response->matches_drawn = playerStatistics.matchesDrawn;

    // No checksum required
}

void CheckersGameMasterNode::joinLobbyRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::JoinLobby::Request> request,
                                              std::shared_ptr<turtle_checkers_interfaces::srv::JoinLobby::Response> response)
{
    response->joined = false;

    uint64_t playerPublicKey = 0u;
    {
        std::lock_guard<std::mutex> lock(m_playerKeysMutex);
        if (m_playerPublicKeys.find(request->player_name) != m_playerPublicKeys.end())
        {
            playerPublicKey = m_playerPublicKeys[request->player_name];
        }
    }

    if (playerPublicKey != 0u)
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
                                                     playerPublicKey,
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

void CheckersGameMasterNode::handleHeartbeats()
{
    // This is run on a separate thread
    while (m_heartbeatThreadRunning)
    {
        // Send out own heartbeat
        auto serverHeartbeatMessage = turtle_checkers_interfaces::msg::ServerHeartbeat();
        serverHeartbeatMessage.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                                               (std::chrono::system_clock::now()).time_since_epoch())
                                               .count();
        serverHeartbeatMessage.checksum_sig = RSAKeyGenerator::createChecksumSignature(
            std::hash<turtle_checkers_interfaces::msg::ServerHeartbeat>{}(serverHeartbeatMessage),
            m_publicKey, m_privateKey);
        m_serverHeartbeatPublisher->publish(serverHeartbeatMessage);

        // Check each player's heartbeat
        auto timeUpTime = std::chrono::system_clock::now() - m_heartbeatTimeout;
        {
            std::vector<std::string> playersToLogout;
            std::lock_guard<std::mutex> lock(m_playerKeysMutex);
            for (const auto &pair : m_playerPublicKeys)
            {
                const auto &playerName = pair.first;
                if (m_playerHeartbeatTimestamps[playerName] < timeUpTime)
                {
                    // Player has timed out
                    playersToLogout.push_back(playerName);
                }
            }

            // Log out any players that have timed out
            for (const auto &playerName : playersToLogout)
            {
                auto playerLoggedOutMessage = turtle_checkers_interfaces::msg::PlayerLoggedOut();
                playerLoggedOutMessage.player_name = playerName;
                playerLoggedOutMessage.checksum_sig = RSAKeyGenerator::createChecksumSignature(
                    std::hash<turtle_checkers_interfaces::msg::PlayerLoggedOut>{}(playerLoggedOutMessage),
                    m_publicKey, m_privateKey);
                m_playerLoggedOutPublisher->publish(playerLoggedOutMessage);

                TurtleLogger::logInfo("Player " + playerName + " has timed out or lost connection");

                m_playerPublicKeys.erase(playerName);
                m_playerHeartbeatTimestamps.erase(playerName);
            }
        }

        std::this_thread::sleep_for(m_heartbeatCheckTimer);
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