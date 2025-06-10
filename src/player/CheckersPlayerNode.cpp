#include "player/CheckersPlayerNode.hpp"

// This prevents a MOC error with versions of boost >= 1.48
// #ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
// #endif

#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp" // For getting the styles directory

#include "turtle_checkers_interfaces/srv/connect_to_game_master.hpp"
#include "turtle_checkers_interfaces/srv/create_account.hpp"
#include "turtle_checkers_interfaces/srv/create_lobby.hpp"
#include "turtle_checkers_interfaces/srv/get_lobby_list.hpp"
#include "turtle_checkers_interfaces/srv/join_lobby.hpp"
#include "turtle_checkers_interfaces/srv/log_in_account.hpp"
#include "turtle_checkers_interfaces/srv/request_piece_move.hpp"
#include "turtle_checkers_interfaces/srv/request_reachable_tiles.hpp"
#include "turtle_checkers_interfaces/msg/declare_winner.hpp"
#include "turtle_checkers_interfaces/msg/draw_declined.hpp"
#include "turtle_checkers_interfaces/msg/draw_offered.hpp"
#include "turtle_checkers_interfaces/msg/forfit.hpp"
#include "turtle_checkers_interfaces/msg/game_start.hpp"
#include "turtle_checkers_interfaces/msg/kick_player.hpp"
#include "turtle_checkers_interfaces/msg/leave_lobby.hpp"
#include "turtle_checkers_interfaces/msg/log_out_account.hpp"
#include "turtle_checkers_interfaces/msg/offer_draw.hpp"
#include "turtle_checkers_interfaces/msg/player_joined_lobby.hpp"
#include "turtle_checkers_interfaces/msg/player_left_lobby.hpp"
#include "turtle_checkers_interfaces/msg/player_readied.hpp"
#include "turtle_checkers_interfaces/msg/player_ready.hpp"
#include "turtle_checkers_interfaces/msg/timer_changed.hpp"
#include "turtle_checkers_interfaces/msg/update_board.hpp"
#include "turtle_checkers_interfaces/msg/update_lobby_owner.hpp"
#include "turtle_checkers_interfaces/msg/update_timer.hpp"

#include <QApplication>
#include <QFile>
#include <QString>
#include <QTimer>

#include <algorithm>
#include <string>
#include <thread>

#include "shared/Hasher.hpp"
#include "shared/RSAKeyGenerator.hpp"
#include "shared/TurtleLogger.hpp"
#include "player/Parameters.hpp"
#include "player/CheckersPlayerWindow.hpp"

using std::placeholders::_1;

CheckersPlayerNode::CheckersPlayerNode(int &argc, char **argv)
    : QApplication(argc, argv)
{
    // Create the node
    rclcpp::init(argc, argv);
    m_playerNode = rclcpp::Node::make_shared("checkers_player_node");

    // Initialize the logger
    TurtleLogger::initialize("checkers_player_node");

    // Create the RSA key pair
    RSAKeyGenerator::generateRSAKeyPair(m_publicKey, m_privateKey);

    // Set the UI styles
    QString stylesPath = (ament_index_cpp::get_package_share_directory("turtle_checkers") +
                          "/styles/checkers_style.qss")
                             .c_str();
    QFile styleFile(stylesPath);
    styleFile.open(QFile::ReadOnly);
    QString styleSheet = QLatin1String(styleFile.readAll());
    setStyleSheet(styleSheet);

    // Create and start the update timer
    m_updateTimer = new QTimer(this);
    m_updateTimer->setInterval(16);
    m_updateTimer->start();
    connect(m_updateTimer, SIGNAL(timeout()), this, SLOT(onUpdate()));
}

CheckersPlayerNode::~CheckersPlayerNode()
{
    delete m_updateTimer;
}

int CheckersPlayerNode::exec()
{
    m_checkersPlayerWindow = std::make_unique<CheckersPlayerWindow>(weak_from_this());
    m_checkersPlayerWindow->show();

    // Try to subscribe to a game and connect to a lobby
    TurtleLogger::logInfo("Player " + Parameters::getPlayerName() + " searching for lobby...");

    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.01f;
    range.to_value = 255.0f;
    rcl_interfaces::msg::ParameterDescriptor boardScaleDescriptor;
    boardScaleDescriptor.description = "Scale of the checkers board render";
    boardScaleDescriptor.floating_point_range.push_back(range);
    m_playerNode->declare_parameter(
        "board_scale", rclcpp::ParameterValue(DEFAULT_BOARD_SCALE), boardScaleDescriptor);

    rcl_interfaces::msg::ParameterDescriptor holonomicDescriptor;
    holonomicDescriptor.description = "If true, then turtles will be holonomic";
    m_playerNode->declare_parameter("holonomic", rclcpp::ParameterValue(false), holonomicDescriptor);

    TurtleLogger::logInfo(std::string("Starting turtle checkers board with node name ") + m_playerNode->get_fully_qualified_name());

    m_connectToGameMasterClient = m_playerNode->create_client<turtle_checkers_interfaces::srv::ConnectToGameMaster>(
        "ConnectToGameMaster");

    m_createAccountClient = m_playerNode->create_client<turtle_checkers_interfaces::srv::CreateAccount>(
        "CreateAccount");
    m_logInAccountClient = m_playerNode->create_client<turtle_checkers_interfaces::srv::LogInAccount>(
        "LogInAccount");

    m_createLobbyClient = m_playerNode->create_client<turtle_checkers_interfaces::srv::CreateLobby>(
        "CreateLobby");
    m_getLobbyListClient = m_playerNode->create_client<turtle_checkers_interfaces::srv::GetLobbyList>(
        "GetLobbyList");
    m_joinLobbyClient = m_playerNode->create_client<turtle_checkers_interfaces::srv::JoinLobby>(
        "JoinLobby");

    m_leaveLobbyPublisher = m_playerNode->create_publisher<turtle_checkers_interfaces::msg::LeaveLobby>(
        "LeaveLobby", 10);
    m_logOutAccountPublisher = m_playerNode->create_publisher<turtle_checkers_interfaces::msg::LogOutAccount>(
        "LogOutAccount", 10);

    // Ensure the game master node is reachable, then get the public key from it
    std::thread(&CheckersPlayerNode::connectToGameMaster, this).detach();

    return QApplication::exec();
}

void CheckersPlayerNode::createLobbyInterfaces(const std::string &lobbyName, const std::string &lobbyId)
{
    // Create the services, subscriptions, and publishers now that we have the full topic names
    m_requestPieceMoveClient = m_playerNode->create_client<turtle_checkers_interfaces::srv::RequestPieceMove>(
        lobbyName + "/id" + lobbyId + "/RequestPieceMove");
    m_requestReachableTilesClient = m_playerNode->create_client<turtle_checkers_interfaces::srv::RequestReachableTiles>(
        lobbyName + "/id" + lobbyId + "/RequestReachableTiles");

    m_declareWinnerSubscription = m_playerNode->create_subscription<turtle_checkers_interfaces::msg::DeclareWinner>(
        lobbyName + "/id" + lobbyId + "/DeclareWinner", 10, std::bind(&CheckersPlayerNode::declareWinnerCallback, this, _1));
    m_drawDeclinedSubscription = m_playerNode->create_subscription<turtle_checkers_interfaces::msg::DrawDeclined>(
        lobbyName + "/id" + lobbyId + "/DrawDeclined", 10, std::bind(&CheckersPlayerNode::drawDeclinedCallback, this, _1));
    m_drawOfferedSubscription = m_playerNode->create_subscription<turtle_checkers_interfaces::msg::DrawOffered>(
        lobbyName + "/id" + lobbyId + "/DrawOffered", 10, std::bind(&CheckersPlayerNode::drawOfferedCallback, this, _1));
    m_gameStartSubscription = m_playerNode->create_subscription<turtle_checkers_interfaces::msg::GameStart>(
        lobbyName + "/id" + lobbyId + "/GameStart", 10, std::bind(&CheckersPlayerNode::gameStartCallback, this, _1));
    m_updateBoardSubscription = m_playerNode->create_subscription<turtle_checkers_interfaces::msg::UpdateBoard>(
        lobbyName + "/id" + lobbyId + "/UpdateBoard", 10, std::bind(&CheckersPlayerNode::updateBoardCallback, this, _1));
    m_playerJoinedLobbySubscription = m_playerNode->create_subscription<turtle_checkers_interfaces::msg::PlayerJoinedLobby>(
        lobbyName + "/id" + lobbyId + "/PlayerJoinedLobby", 10, std::bind(&CheckersPlayerNode::playerJoinedLobbyCallback, this, _1));
    m_playerLeftLobbySubscription = m_playerNode->create_subscription<turtle_checkers_interfaces::msg::PlayerLeftLobby>(
        lobbyName + "/id" + lobbyId + "/PlayerLeftLobby", 10, std::bind(&CheckersPlayerNode::playerLeftLobbyCallback, this, _1));
    m_playerReadiedSubscription = m_playerNode->create_subscription<turtle_checkers_interfaces::msg::PlayerReadied>(
        lobbyName + "/id" + lobbyId + "/PlayerReadied", 10, std::bind(&CheckersPlayerNode::playerReadiedCallback, this, _1));
    m_updateLobbyOwnerSubscription = m_playerNode->create_subscription<turtle_checkers_interfaces::msg::UpdateLobbyOwner>(
        lobbyName + "/id" + lobbyId + "/UpdateLobbyOwner", 10, std::bind(&CheckersPlayerNode::updateLobbyOwnerCallback, this, _1));
    m_updateTimerSubscription = m_playerNode->create_subscription<turtle_checkers_interfaces::msg::UpdateTimer>(
        lobbyName + "/id" + lobbyId + "/UpdateTimer", 10, std::bind(&CheckersPlayerNode::updateTimerCallback, this, _1));

    m_forfitPublisher = m_playerNode->create_publisher<turtle_checkers_interfaces::msg::Forfit>(
        lobbyName + "/id" + lobbyId + "/Forfit", 10);
    m_kickPlayerPublisher = m_playerNode->create_publisher<turtle_checkers_interfaces::msg::KickPlayer>(
        lobbyName + "/id" + lobbyId + "/KickPlayer", 10);
    m_offerDrawPublisher = m_playerNode->create_publisher<turtle_checkers_interfaces::msg::OfferDraw>(
        lobbyName + "/id" + lobbyId + "/OfferDraw", 10);
    m_playerReadyPublisher = m_playerNode->create_publisher<turtle_checkers_interfaces::msg::PlayerReady>(
        lobbyName + "/id" + lobbyId + "/PlayerReady", 10);
    m_timerChangedPublisher = m_playerNode->create_publisher<turtle_checkers_interfaces::msg::TimerChanged>(
        lobbyName + "/id" + lobbyId + "/TimerChanged", 10);
}

void CheckersPlayerNode::parameterEventCallback(
    const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr event)
{
    // Only consider events from this node
    if (event->node == m_playerNode->get_fully_qualified_name())
    {
        // Since parameter events for this event aren't expected frequently just always call update()
        if (m_checkersPlayerWindow)
        {
            m_checkersPlayerWindow->update();
        }
    }
}

void CheckersPlayerNode::connectToGameMaster()
{
    bool alive = true;
    while (alive && !m_connectToGameMasterClient->wait_for_service(std::chrono::seconds(10)))
    {
        if (!rclcpp::ok())
        {
            TurtleLogger::logError("Interrupted while waiting for the service. Exiting.");
            alive = false;
            QApplication::quit(); // Stop running the application and exit
        }
        TurtleLogger::logInfo("Service not available, waiting again...");
    }
    if (alive)
    {
        auto request = std::make_shared<turtle_checkers_interfaces::srv::ConnectToGameMaster::Request>();
        m_connectToGameMasterClient->async_send_request(request,
                                                        std::bind(&CheckersPlayerNode::connectToGameMasterResponse,
                                                                  this, std::placeholders::_1));
    }
}

void CheckersPlayerNode::createAccount(
    const std::string &playerName,
    const std::string &playerPassword)
{
    auto request = std::make_shared<turtle_checkers_interfaces::srv::CreateAccount::Request>();
    request->player_name = playerName;
    uint64_t hashedPlayerPassword = RSAKeyGenerator::hashString(playerPassword);
    uint64_t encryptedHashedPlayerPassword = RSAKeyGenerator::encrypt(hashedPlayerPassword, m_gameMasterPublicKey);
    request->encrypted_hashed_player_password = encryptedHashedPlayerPassword;
    request->player_public_key = m_publicKey;
    // No signature
    m_createAccountClient->async_send_request(request,
                                              std::bind(&CheckersPlayerNode::createAccountResponse,
                                                        this, std::placeholders::_1));
}

void CheckersPlayerNode::logInAccount(
    const std::string &playerName,
    const std::string &playerPassword)
{
    auto request = std::make_shared<turtle_checkers_interfaces::srv::LogInAccount::Request>();
    request->player_name = playerName;
    uint64_t hashedPlayerPassword = RSAKeyGenerator::hashString(playerPassword);
    uint64_t encryptedHashedPlayerPassword = RSAKeyGenerator::encrypt(hashedPlayerPassword, m_gameMasterPublicKey);
    request->encrypted_hashed_player_password = encryptedHashedPlayerPassword;
    request->player_public_key = m_publicKey;
    // No signature
    m_logInAccountClient->async_send_request(request,
                                             std::bind(&CheckersPlayerNode::logInAccountResponse,
                                                       this, std::placeholders::_1));
}

void CheckersPlayerNode::logOutAccount()
{
    // Tell the game master we've logged out
    auto message = turtle_checkers_interfaces::msg::LogOutAccount();
    message.player_name = Parameters::getPlayerName();
    message.checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::msg::LogOutAccount>{}(message),
        m_publicKey, m_privateKey);
    m_logOutAccountPublisher->publish(message);
}

void CheckersPlayerNode::createLobby(
    const std::string &playerName,
    const std::string &lobbyName,
    const std::string &lobbyPassword,
    TurtlePieceColor playerDesiredColor)
{
    auto request = std::make_shared<turtle_checkers_interfaces::srv::CreateLobby::Request>();
    request->player_name = playerName;
    request->lobby_name = lobbyName;
    if (lobbyPassword.empty())
    {
        request->encrypted_hashed_lobby_password = 0u;
    }
    else
    {
        uint64_t hashedLobbyPassword = std::hash<std::string>{}(lobbyPassword);
        uint64_t encryptedHashedLobbyPassword = RSAKeyGenerator::encrypt(hashedLobbyPassword, m_gameMasterPublicKey);
        request->encrypted_hashed_lobby_password = encryptedHashedLobbyPassword;
    }
    request->desired_player_color = static_cast<size_t>(playerDesiredColor);
    request->checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::srv::CreateLobby::Request::SharedPtr>{}(request),
        m_publicKey, m_privateKey);
    m_createLobbyClient->async_send_request(request,
                                            std::bind(&CheckersPlayerNode::createLobbyResponse,
                                                      this, std::placeholders::_1));
}

void CheckersPlayerNode::joinLobby(
    const std::string &playerName,
    const std::string &lobbyName,
    const std::string &lobbyId,
    const std::string &lobbyPassword,
    TurtlePieceColor playerDesiredColor)
{
    auto request = std::make_shared<turtle_checkers_interfaces::srv::JoinLobby::Request>();
    request->player_name = playerName;
    request->lobby_name = lobbyName;
    request->lobby_id = lobbyId;
    if (lobbyPassword.empty())
    {
        request->encrypted_hashed_lobby_password = 0u;
    }
    else
    {
        uint64_t hashedLobbyPassword = std::hash<std::string>{}(lobbyPassword);
        uint64_t encryptedHashedLobbyPassword = RSAKeyGenerator::encrypt(hashedLobbyPassword, m_gameMasterPublicKey);
        request->encrypted_hashed_lobby_password = encryptedHashedLobbyPassword;
    }
    request->desired_player_color = static_cast<size_t>(playerDesiredColor);
    request->checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::srv::JoinLobby::Request::SharedPtr>{}(request),
        m_publicKey, m_privateKey);
    m_joinLobbyClient->async_send_request(request,
                                          std::bind(&CheckersPlayerNode::joinLobbyResponse,
                                                    this, std::placeholders::_1));
}

void CheckersPlayerNode::getLobbyList()
{
    auto request = std::make_shared<turtle_checkers_interfaces::srv::GetLobbyList::Request>();
    // No signature required
    m_getLobbyListClient->async_send_request(request,
                                             std::bind(&CheckersPlayerNode::getLobbyListResponse,
                                                       this, std::placeholders::_1));
}

void CheckersPlayerNode::leaveLobby()
{
    // Tell the game master we've left the lobby
    auto message = turtle_checkers_interfaces::msg::LeaveLobby();
    message.lobby_name = Parameters::getLobbyName();
    message.lobby_id = Parameters::getLobbyId();
    message.player_name = Parameters::getPlayerName();
    message.checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::msg::LeaveLobby>{}(message),
        m_publicKey, m_privateKey);
    m_leaveLobbyPublisher->publish(message);
}

void CheckersPlayerNode::kickPlayer(const std::string &playerName)
{
    // Tell the game master to kick a player
    auto message = turtle_checkers_interfaces::msg::KickPlayer();
    message.lobby_name = Parameters::getLobbyName();
    message.lobby_id = Parameters::getLobbyId();
    message.requesting_player_name = Parameters::getPlayerName();
    message.kick_player_name = playerName;
    message.checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::msg::KickPlayer>{}(message),
        m_publicKey, m_privateKey);
    m_kickPlayerPublisher->publish(message);
}

void CheckersPlayerNode::setReady(bool ready)
{
    // Tell the game master if this player is ready or not
    auto message = turtle_checkers_interfaces::msg::PlayerReady();
    message.lobby_name = Parameters::getLobbyName();
    message.lobby_id = Parameters::getLobbyId();
    message.player_name = Parameters::getPlayerName();
    message.ready = ready;
    message.checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::msg::PlayerReady>{}(message),
        m_publicKey, m_privateKey);
    m_playerReadyPublisher->publish(message);
}

void CheckersPlayerNode::setTimer(uint64_t timerSeconds)
{
    // Tell the game master the new timer
    auto message = turtle_checkers_interfaces::msg::TimerChanged();
    message.lobby_name = Parameters::getLobbyName();
    message.lobby_id = Parameters::getLobbyId();
    message.player_name = Parameters::getPlayerName();
    message.timer_seconds = timerSeconds;
    message.checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::msg::TimerChanged>{}(message),
        m_publicKey, m_privateKey);
    m_timerChangedPublisher->publish(message);
}

void CheckersPlayerNode::connectToGameMasterResponse(rclcpp::Client<turtle_checkers_interfaces::srv::ConnectToGameMaster>::SharedFuture future)
{
    auto result = future.get();

    // No checksum to check

    m_gameMasterPublicKey = result->game_master_public_key;

    m_checkersPlayerWindow->setConnectedToServer(true);

    m_checkersPlayerWindow->update();
}

void CheckersPlayerNode::createAccountResponse(rclcpp::Client<turtle_checkers_interfaces::srv::CreateAccount>::SharedFuture future)
{
    auto result = future.get();

    if (!RSAKeyGenerator::checksumSignatureMatches(
            std::hash<turtle_checkers_interfaces::srv::CreateAccount::Response::SharedPtr>{}(result),
            m_gameMasterPublicKey, result->checksum_sig))
    {
        std::cerr << "Checksum failed" << std::endl;
        return; // Checksum did not match with the public key
    }

    if (result->created)
    {
        m_checkersPlayerWindow->accountCreated(result->player_name);
    }
    else
    {
        m_checkersPlayerWindow->failedCreate(result->error_msg);
        TurtleLogger::logWarn(result->error_msg);
    }
}

void CheckersPlayerNode::createLobbyResponse(rclcpp::Client<turtle_checkers_interfaces::srv::CreateLobby>::SharedFuture future)
{
    auto result = future.get();

    if (!RSAKeyGenerator::checksumSignatureMatches(
            std::hash<turtle_checkers_interfaces::srv::CreateLobby::Response::SharedPtr>{}(result),
            m_gameMasterPublicKey, result->checksum_sig))
    {
        std::cerr << "Checksum failed" << std::endl;
        return; // Checksum did not match with the public key
    }

    if (result->created)
    {
        TurtlePieceColor lobbyOwnerPlayerColor = TurtlePieceColor::None;
        if (Parameters::getPlayerName() == result->black_player_name)
        {
            lobbyOwnerPlayerColor = TurtlePieceColor::Black;
        }
        else if (Parameters::getPlayerName() == result->red_player_name)
        {
            lobbyOwnerPlayerColor = TurtlePieceColor::Red;
        }
        createLobbyInterfaces(result->lobby_name, result->lobby_id);
        m_checkersPlayerWindow->connectedToLobby(result->lobby_name,
                                                 result->lobby_id,
                                                 result->black_player_name,
                                                 result->red_player_name,
                                                 lobbyOwnerPlayerColor,
                                                 result->black_player_ready,
                                                 result->red_player_ready,
                                                 0u);
    }
    else
    {
        TurtleLogger::logWarn(result->error_msg);
    }
}

void CheckersPlayerNode::getLobbyListResponse(rclcpp::Client<turtle_checkers_interfaces::srv::GetLobbyList>::SharedFuture future)
{
    auto result = future.get();

    if (!RSAKeyGenerator::checksumSignatureMatches(
            std::hash<turtle_checkers_interfaces::srv::GetLobbyList::Response::SharedPtr>{}(result),
            m_gameMasterPublicKey, result->checksum_sig))
    {
        std::cerr << "Checksum failed" << std::endl;
        return; // Checksum did not match with the public key
    }

    m_checkersPlayerWindow->updateLobbyList(result->lobby_names,
                                            result->lobby_ids,
                                            result->has_passwords,
                                            result->joined_black_player_names,
                                            result->joined_red_player_names);
}

void CheckersPlayerNode::joinLobbyResponse(rclcpp::Client<turtle_checkers_interfaces::srv::JoinLobby>::SharedFuture future)
{
    auto result = future.get();

    if (!RSAKeyGenerator::checksumSignatureMatches(
            std::hash<turtle_checkers_interfaces::srv::JoinLobby::Response::SharedPtr>{}(result),
            m_gameMasterPublicKey, result->checksum_sig))
    {
        std::cerr << "Checksum failed" << std::endl;
        return; // Checksum did not match with the public key
    }

    if (result->joined)
    {
        TurtlePieceColor lobbyOwnerPlayerColor = TurtlePieceColor::None;
        if (Parameters::getPlayerName() == result->black_player_name)
        {
            lobbyOwnerPlayerColor = TurtlePieceColor::Red;
        }
        else if (Parameters::getPlayerName() == result->red_player_name)
        {
            lobbyOwnerPlayerColor = TurtlePieceColor::Black;
        }
        createLobbyInterfaces(result->lobby_name, result->lobby_id);
        m_checkersPlayerWindow->connectedToLobby(result->lobby_name,
                                                 result->lobby_id,
                                                 result->black_player_name,
                                                 result->red_player_name,
                                                 lobbyOwnerPlayerColor,
                                                 result->black_player_ready,
                                                 result->red_player_ready,
                                                 result->timer_seconds);
    }
    else
    {
        if (result->error_msg == "Incorrect password.") // TODO
        {
            m_checkersPlayerWindow->setPasswordIncorrect();
        }
        TurtleLogger::logWarn(result->error_msg);
    }
}

void CheckersPlayerNode::logInAccountResponse(rclcpp::Client<turtle_checkers_interfaces::srv::LogInAccount>::SharedFuture future)
{
    auto result = future.get();

    if (!RSAKeyGenerator::checksumSignatureMatches(
            std::hash<turtle_checkers_interfaces::srv::LogInAccount::Response::SharedPtr>{}(result),
            m_gameMasterPublicKey, result->checksum_sig))
    {
        std::cerr << "Checksum failed" << std::endl;
        return; // Checksum did not match with the public key
    }

    if (result->logged_in)
    {
        m_checkersPlayerWindow->loggedIn(result->player_name);
    }
    else
    {
        m_checkersPlayerWindow->failedLogIn(result->error_msg);
        TurtleLogger::logWarn(result->error_msg);
    }
}

void CheckersPlayerNode::requestPieceMoveResponse(rclcpp::Client<turtle_checkers_interfaces::srv::RequestPieceMove>::SharedFuture future)
{
    auto result = future.get();

    if (!RSAKeyGenerator::checksumSignatureMatches(
            std::hash<turtle_checkers_interfaces::srv::RequestPieceMove::Response::SharedPtr>{}(result),
            m_gameMasterPublicKey, result->checksum_sig))
    {
        std::cerr << "Checksum failed" << std::endl;
        return; // Checksum did not match with the public key
    }

    m_checkersPlayerWindow->requestedPieceMoveAccepted(result->move_accepted);

    m_checkersPlayerWindow->update();
}

void CheckersPlayerNode::requestReachableTilesResponse(rclcpp::Client<turtle_checkers_interfaces::srv::RequestReachableTiles>::SharedFuture future)
{
    auto result = future.get();

    if (!RSAKeyGenerator::checksumSignatureMatches(
            std::hash<turtle_checkers_interfaces::srv::RequestReachableTiles::Response::SharedPtr>{}(result),
            m_gameMasterPublicKey, result->checksum_sig))
    {
        std::cerr << "Checksum failed" << std::endl;
        return; // Checksum did not match with the public key
    }

    const auto &reachableTileIndices = result->reachable_tile_indices;
    m_checkersPlayerWindow->requestedReachableTiles(reachableTileIndices);

    m_checkersPlayerWindow->update();
}

void CheckersPlayerNode::declareWinnerCallback(const turtle_checkers_interfaces::msg::DeclareWinner::SharedPtr message)
{
    if (Parameters::getLobbyName() != message->lobby_name || Parameters::getLobbyId() != message->lobby_id)
    {
        return; // Wrong lobby
    }

    if (!RSAKeyGenerator::checksumSignatureMatches(
            std::hash<turtle_checkers_interfaces::msg::DeclareWinner>{}(*message),
            m_gameMasterPublicKey, message->checksum_sig))
    {
        std::cerr << "Checksum failed" << std::endl;
        return; // Checksum did not match with the public key
    }

    Winner winner = static_cast<Winner>(message->winner);
    m_checkersPlayerWindow->declaredWinner(winner);

    m_checkersPlayerWindow->update();
}

void CheckersPlayerNode::drawDeclinedCallback(const turtle_checkers_interfaces::msg::DrawDeclined::SharedPtr message)
{
    if (Parameters::getLobbyName() != message->lobby_name || Parameters::getLobbyId() != message->lobby_id)
    {
        return;
    }

    if (!RSAKeyGenerator::checksumSignatureMatches(
            std::hash<turtle_checkers_interfaces::msg::DrawDeclined>{}(*message),
            m_gameMasterPublicKey, message->checksum_sig))
    {
        std::cerr << "Checksum failed" << std::endl;
        return; // Checksum did not match with the public key
    }

    m_checkersPlayerWindow->drawDeclined();
}

void CheckersPlayerNode::drawOfferedCallback(const turtle_checkers_interfaces::msg::DrawOffered::SharedPtr message)
{
    if (Parameters::getLobbyName() != message->lobby_name || Parameters::getLobbyId() != message->lobby_id)
    {
        return;
    }

    if (!RSAKeyGenerator::checksumSignatureMatches(
            std::hash<turtle_checkers_interfaces::msg::DrawOffered>{}(*message),
            m_gameMasterPublicKey, message->checksum_sig))
    {
        std::cerr << "Checksum failed" << std::endl;
        return; // Checksum did not match with the public key
    }

    if (Parameters::getPlayerName() == message->player_name)
    {
        // Do not respond to own offer for a draw
        return;
    }

    m_checkersPlayerWindow->drawOffered();
}

void CheckersPlayerNode::gameStartCallback(const turtle_checkers_interfaces::msg::GameStart::SharedPtr message)
{
    if (Parameters::getLobbyName() != message->lobby_name || Parameters::getLobbyId() != message->lobby_id)
    {
        return;
    }

    if (!RSAKeyGenerator::checksumSignatureMatches(
            std::hash<turtle_checkers_interfaces::msg::GameStart>{}(*message),
            m_gameMasterPublicKey, message->checksum_sig))
    {
        std::cerr << "Checksum failed" << std::endl;
        return; // Checksum did not match with the public key
    }

    GameState gameState = static_cast<GameState>(message->game_state);
    const auto &movableTileIndices = message->movable_tile_indices;
    auto blackTimeRemainSec = message->black_time_remaining_seconds;
    auto redTimeRemainSec = message->red_time_remaining_seconds;
    m_checkersPlayerWindow->gameStarted(gameState, movableTileIndices, blackTimeRemainSec, redTimeRemainSec);

    m_checkersPlayerWindow->update();
}

void CheckersPlayerNode::playerJoinedLobbyCallback(const turtle_checkers_interfaces::msg::PlayerJoinedLobby::SharedPtr message)
{
    if (Parameters::getLobbyName() != message->lobby_name || Parameters::getLobbyId() != message->lobby_id)
    {
        return;
    }

    if (!RSAKeyGenerator::checksumSignatureMatches(
            std::hash<turtle_checkers_interfaces::msg::PlayerJoinedLobby>{}(*message),
            m_gameMasterPublicKey, message->checksum_sig))
    {
        std::cerr << "Checksum failed" << std::endl;
        return; // Checksum did not match with the public key
    }

    m_checkersPlayerWindow->playerJoinedLobby(message->player_name, static_cast<TurtlePieceColor>(message->player_color));
}

void CheckersPlayerNode::playerLeftLobbyCallback(const turtle_checkers_interfaces::msg::PlayerLeftLobby::SharedPtr message)
{
    if (Parameters::getLobbyName() != message->lobby_name || Parameters::getLobbyId() != message->lobby_id)
    {
        return;
    }

    if (!RSAKeyGenerator::checksumSignatureMatches(
            std::hash<turtle_checkers_interfaces::msg::PlayerLeftLobby>{}(*message),
            m_gameMasterPublicKey, message->checksum_sig))
    {
        std::cerr << "Checksum failed" << std::endl;
        return; // Checksum did not match with the public key
    }

    m_checkersPlayerWindow->playerLeftLobby(message->player_name);
}

void CheckersPlayerNode::playerReadiedCallback(const turtle_checkers_interfaces::msg::PlayerReadied::SharedPtr message)
{
    if (Parameters::getLobbyName() != message->lobby_name || Parameters::getLobbyId() != message->lobby_id)
    {
        return;
    }

    if (!RSAKeyGenerator::checksumSignatureMatches(
            std::hash<turtle_checkers_interfaces::msg::PlayerReadied>{}(*message),
            m_gameMasterPublicKey, message->checksum_sig))
    {
        std::cerr << "Checksum failed" << std::endl;
        return; // Checksum did not match with the public key
    }

    m_checkersPlayerWindow->setPlayerReady(message->player_name, message->ready);
}

void CheckersPlayerNode::updateBoardCallback(const turtle_checkers_interfaces::msg::UpdateBoard::SharedPtr message)
{
    if (Parameters::getLobbyName() != message->lobby_name || Parameters::getLobbyId() != message->lobby_id)
    {
        return;
    }

    if (!RSAKeyGenerator::checksumSignatureMatches(
            std::hash<turtle_checkers_interfaces::msg::UpdateBoard>{}(*message),
            m_gameMasterPublicKey, message->checksum_sig))
    {
        std::cerr << "Checksum failed" << std::endl;
        return; // Checksum did not match with the public key
    }

    auto sourceTileIndex = message->source_tile_index;
    auto destinationTileIndex = message->destination_tile_index;
    auto gameState = static_cast<GameState>(message->game_state);
    auto slainPieceTileIndex = message->slain_piece_tile_index;
    auto kingPiece = message->king_piece;
    const auto &movableTileIndices = message->movable_tile_indices;
    auto blackTimeRemainSec = message->black_time_remaining_seconds;
    auto redTimeRemainSec = message->red_time_remaining_seconds;

    m_checkersPlayerWindow->updatedBoard(sourceTileIndex, destinationTileIndex, gameState,
                                         slainPieceTileIndex, kingPiece, movableTileIndices,
                                         blackTimeRemainSec, redTimeRemainSec);

    m_checkersPlayerWindow->update();
}

void CheckersPlayerNode::updateLobbyOwnerCallback(const turtle_checkers_interfaces::msg::UpdateLobbyOwner::SharedPtr message)
{
    if (Parameters::getLobbyName() != message->lobby_name || Parameters::getLobbyId() != message->lobby_id)
    {
        return;
    }

    if (!RSAKeyGenerator::checksumSignatureMatches(
            std::hash<turtle_checkers_interfaces::msg::UpdateLobbyOwner>{}(*message),
            m_gameMasterPublicKey, message->checksum_sig))
    {
        std::cerr << "Checksum failed" << std::endl;
        return; // Checksum did not match with the public key
    }

    m_checkersPlayerWindow->updateLobbyOwner(message->lobby_owner_player_name);
}

void CheckersPlayerNode::updateTimerCallback(const turtle_checkers_interfaces::msg::UpdateTimer::SharedPtr message)
{
    if (Parameters::getLobbyName() != message->lobby_name || Parameters::getLobbyId() != message->lobby_id)
    {
        return;
    }

    if (!RSAKeyGenerator::checksumSignatureMatches(
            std::hash<turtle_checkers_interfaces::msg::UpdateTimer>{}(*message),
            m_gameMasterPublicKey, message->checksum_sig))
    {
        std::cerr << "Checksum failed" << std::endl;
        return; // Checksum did not match with the public key
    }

    m_checkersPlayerWindow->updateTimer(message->timer_seconds);
}

void CheckersPlayerNode::requestPieceMove(size_t sourceTileIndex, size_t destinationTileIndex)
{
    auto request = std::make_shared<turtle_checkers_interfaces::srv::RequestPieceMove::Request>();
    request->lobby_name = Parameters::getLobbyName();
    request->lobby_id = Parameters::getLobbyId();
    request->source_tile_index = sourceTileIndex;
    request->destination_tile_index = destinationTileIndex;
    request->checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::srv::RequestPieceMove::Request::SharedPtr>{}(request),
        m_publicKey, m_privateKey);
    m_requestPieceMoveClient->async_send_request(request,
                                                 std::bind(&CheckersPlayerNode::requestPieceMoveResponse,
                                                           this, std::placeholders::_1));

    m_checkersPlayerWindow->update();
}

void CheckersPlayerNode::requestReachableTiles(size_t selectedPieceTileIndex)
{
    auto request = std::make_shared<turtle_checkers_interfaces::srv::RequestReachableTiles::Request>();
    request->lobby_name = Parameters::getLobbyName();
    request->lobby_id = Parameters::getLobbyId();
    request->tile_index = selectedPieceTileIndex;
    request->checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::srv::RequestReachableTiles::Request::SharedPtr>{}(request),
        m_publicKey, m_privateKey);
    m_requestReachableTilesClient->async_send_request(request,
                                                      std::bind(&CheckersPlayerNode::requestReachableTilesResponse,
                                                                this, std::placeholders::_1));

    m_checkersPlayerWindow->update();
}

void CheckersPlayerNode::offerDraw()
{
    auto message = turtle_checkers_interfaces::msg::OfferDraw();
    message.lobby_name = Parameters::getLobbyName();
    message.lobby_id = Parameters::getLobbyId();
    message.player_name = Parameters::getPlayerName();
    message.accept_draw = true;
    message.checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::msg::OfferDraw>{}(message),
        m_publicKey, m_privateKey);
    m_offerDrawPublisher->publish(message);
}

void CheckersPlayerNode::declineDraw()
{
    auto message = turtle_checkers_interfaces::msg::OfferDraw();
    message.lobby_name = Parameters::getLobbyName();
    message.lobby_id = Parameters::getLobbyId();
    message.player_name = Parameters::getPlayerName();
    message.accept_draw = false;
    message.checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::msg::OfferDraw>{}(message),
        m_publicKey, m_privateKey);
    m_offerDrawPublisher->publish(message);
}

void CheckersPlayerNode::forfit()
{
    auto message = turtle_checkers_interfaces::msg::Forfit();
    message.lobby_name = Parameters::getLobbyName();
    message.lobby_id = Parameters::getLobbyId();
    message.player_name = Parameters::getPlayerName();
    message.checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::msg::Forfit>{}(message),
        m_publicKey, m_privateKey);
    m_forfitPublisher->publish(message);
}

void CheckersPlayerNode::onUpdate()
{
    if (!rclcpp::ok())
    {
        quit();
        return;
    }

    rclcpp::spin_some(m_playerNode);
}

void CheckersPlayerNode::shutdown()
{
    rclcpp::shutdown();
}

std::shared_ptr<rclcpp::Node> &CheckersPlayerNode::getNodeHandle()
{
    return m_playerNode;
}

int main(int argc, char *argv[])
{
    auto playerNode = std::make_shared<CheckersPlayerNode>(argc, argv);
    return playerNode->exec();
}