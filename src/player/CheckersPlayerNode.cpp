#include "player/CheckersPlayerNode.hpp"

// This prevents a MOC error with versions of boost >= 1.48
// #ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
// #endif

#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp" // For getting the styles directory

#include "turtle_checkers_interfaces/srv/connect_to_game_master.hpp"
#include "turtle_checkers_interfaces/srv/create_lobby.hpp"
#include "turtle_checkers_interfaces/srv/get_lobby_list.hpp"
#include "turtle_checkers_interfaces/srv/join_lobby.hpp"
#include "turtle_checkers_interfaces/srv/request_piece_move.hpp"
#include "turtle_checkers_interfaces/srv/request_reachable_tiles.hpp"
#include "turtle_checkers_interfaces/msg/declare_winner.hpp"
#include "turtle_checkers_interfaces/msg/draw_declined.hpp"
#include "turtle_checkers_interfaces/msg/draw_offered.hpp"
#include "turtle_checkers_interfaces/msg/forfit.hpp"
#include "turtle_checkers_interfaces/msg/game_start.hpp"
#include "turtle_checkers_interfaces/msg/leave_lobby.hpp"
#include "turtle_checkers_interfaces/msg/offer_draw.hpp"
#include "turtle_checkers_interfaces/msg/player_joined_lobby.hpp"
#include "turtle_checkers_interfaces/msg/player_left_lobby.hpp"
#include "turtle_checkers_interfaces/msg/player_readied.hpp"
#include "turtle_checkers_interfaces/msg/player_ready.hpp"
#include "turtle_checkers_interfaces/msg/update_board.hpp"

#include <QApplication>
#include <QFile>
#include <QString>
#include <QTimer>

#include <algorithm>

#include "shared/Hasher.hpp"
#include "shared/RSAKeyGenerator.hpp"
#include "player/CheckersPlayerWindow.hpp"

using std::placeholders::_1;

CheckersPlayerNode::CheckersPlayerNode(int &argc, char **argv)
    : QApplication(argc, argv)
{
    m_playerName = "";
    m_lobbyName = "";
    m_lobbyId = "";

    // Create the RSA key pair
    RSAKeyGenerator::generateRSAKeyPair(m_publicKey, m_privateKey);

    // Create the node
    rclcpp::init(argc, argv);
    m_playerNode = rclcpp::Node::make_shared("checkers_player_node");

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
    RCLCPP_INFO(m_playerNode->get_logger(), "Player " + m_playerName + " searching for lobby...");

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

    RCLCPP_INFO(
        m_playerNode->get_logger(), "Starting turtle checkers board with node name %s", m_playerNode->get_fully_qualified_name());

    m_connectToGameMasterClient = m_playerNode->create_client<turtle_checkers_interfaces::srv::ConnectToGameMaster>(
        "ConnectToGameMaster");

    m_createLobbyClient = m_playerNode->create_client<turtle_checkers_interfaces::srv::CreateLobby>(
        "CreateLobby");
    m_getLobbyListClient = m_playerNode->create_client<turtle_checkers_interfaces::srv::GetLobbyList>(
        "GetLobbyList");
    m_joinLobbyClient = m_playerNode->create_client<turtle_checkers_interfaces::srv::JoinLobby>(
        "JoinLobby");

    m_leaveLobbyPublisher = m_playerNode->create_publisher<turtle_checkers_interfaces::msg::LeaveLobby>(
        "LeaveLobby", 10);

    // Ensure the game master node is reachable, then get the public key from it
    // TODO - We should refactor the menus so we enter a name before connecting
    while (!m_connectToGameMasterClient->wait_for_service(std::chrono::seconds(10)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }
    auto request = std::make_shared<turtle_checkers_interfaces::srv::ConnectToGameMaster::Request>();
    m_connectToGameMasterClient->async_send_request(request,
                                                    std::bind(&CheckersPlayerNode::connectToGameMasterResponse,
                                                              this, std::placeholders::_1));

    return QApplication::exec();
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

void CheckersPlayerNode::createLobby(
    const std::string &playerName,
    const std::string &lobbyName,
    TurtlePieceColor playerDesiredColor)
{
    m_playerName = playerName;
    m_lobbyName = lobbyName;
    auto request = std::make_shared<turtle_checkers_interfaces::srv::CreateLobby::Request>();
    request->player_name = m_playerName;
    request->lobby_name = m_lobbyName;
    request->desired_player_color = static_cast<size_t>(playerDesiredColor);
    request->player_public_key = m_publicKey;
    m_createLobbyClient->async_send_request(request,
                                            std::bind(&CheckersPlayerNode::createLobbyResponse,
                                                      this, std::placeholders::_1));
}

void CheckersPlayerNode::joinLobby(
    const std::string &playerName,
    const std::string &lobbyName,
    const std::string &lobbyId,
    TurtlePieceColor playerDesiredColor)
{
    m_playerName = playerName;
    m_lobbyName = lobbyName;
    m_lobbyId = lobbyId;
    auto request = std::make_shared<turtle_checkers_interfaces::srv::JoinLobby::Request>();
    request->player_name = m_playerName;
    request->lobby_name = m_lobbyName;
    request->lobby_id = m_lobbyId;
    request->desired_player_color = static_cast<size_t>(playerDesiredColor);
    request->player_public_key = m_publicKey;
    m_joinLobbyClient->async_send_request(request,
                                          std::bind(&CheckersPlayerNode::joinLobbyResponse,
                                                    this, std::placeholders::_1));
}

void CheckersPlayerNode::getLobbyList()
{
    auto request = std::make_shared<turtle_checkers_interfaces::srv::GetLobbyList::Request>();
    m_getLobbyListClient->async_send_request(request,
                                             std::bind(&CheckersPlayerNode::getLobbyListResponse,
                                                       this, std::placeholders::_1));
}

void CheckersPlayerNode::leaveLobby()
{
    // Tell the game master we've left the lobby
    auto message = turtle_checkers_interfaces::msg::LeaveLobby();
    message.lobby_name = m_lobbyName;
    message.lobby_id = m_lobbyId;
    message.player_name = m_playerName;
    m_leaveLobbyPublisher->publish(message);
}

void CheckersPlayerNode::setReady(bool ready)
{
    // Tell the game master if this player is ready or not
    auto message = turtle_checkers_interfaces::msg::PlayerReady();
    message.lobby_name = m_lobbyName;
    message.lobby_id = m_lobbyId;
    message.player_name = m_playerName;
    message.ready = ready;
    m_playerReadyPublisher->publish(message);
}

void CheckersPlayerNode::connectToGameMasterResponse(rclcpp::Client<turtle_checkers_interfaces::srv::ConnectToGameMaster>::SharedFuture future)
{
    auto result = future.get();

    m_gameMasterPublicKey = result->game_master_public_key;

    m_checkersPlayerWindow->update();
}

void CheckersPlayerNode::createLobbyResponse(rclcpp::Client<turtle_checkers_interfaces::srv::CreateLobby>::SharedFuture future)
{
    auto result = future.get();

    if (result->created)
    {
        createLobbyInterfaces(result->lobby_name, result->lobby_id);
        m_checkersPlayerWindow->connectedToLobby(result->lobby_name,
                                                 result->lobby_id,
                                                 result->black_player_name,
                                                 result->red_player_name,
                                                 result->black_player_ready,
                                                 result->red_player_ready);
    }
    else
    {
        RCLCPP_WARN(m_playerNode->get_logger(), result->error_msg);
    }
}

void CheckersPlayerNode::getLobbyListResponse(rclcpp::Client<turtle_checkers_interfaces::srv::GetLobbyList>::SharedFuture future)
{
    auto result = future.get();

    m_checkersPlayerWindow->updateLobbyList(result->lobby_names,
                                            result->lobby_ids,
                                            result->joined_black_player_names,
                                            result->joined_red_player_names);
}

void CheckersPlayerNode::joinLobbyResponse(rclcpp::Client<turtle_checkers_interfaces::srv::JoinLobby>::SharedFuture future)
{
    auto result = future.get();

    if (result->joined)
    {
        createLobbyInterfaces(result->lobby_name, result->lobby_id);
        m_checkersPlayerWindow->connectedToLobby(result->lobby_name,
                                                 result->lobby_id,
                                                 result->black_player_name,
                                                 result->red_player_name,
                                                 result->black_player_ready,
                                                 result->red_player_ready);
    }
    else
    {
        RCLCPP_WARN(m_playerNode->get_logger(), result->error_msg);
    }
}

void CheckersPlayerNode::createLobbyInterfaces(const std::string &lobbyName, const std::string &lobbyId)
{
    m_lobbyName = lobbyName;
    m_lobbyId = lobbyId;

    // Create the services, subscriptions, and publishers now that we have the full topic names
    m_requestPieceMoveClient = m_playerNode->create_client<turtle_checkers_interfaces::srv::RequestPieceMove>(
        m_lobbyName + "/id" + m_lobbyId + "/RequestPieceMove");
    m_requestReachableTilesClient = m_playerNode->create_client<turtle_checkers_interfaces::srv::RequestReachableTiles>(
        m_lobbyName + "/id" + m_lobbyId + "/RequestReachableTiles");

    m_declareWinnerSubscription = m_playerNode->create_subscription<turtle_checkers_interfaces::msg::DeclareWinner>(
        m_lobbyName + "/id" + m_lobbyId + "/DeclareWinner", 10, std::bind(&CheckersPlayerNode::declareWinnerCallback, this, _1));
    m_drawDeclinedSubscription = m_playerNode->create_subscription<turtle_checkers_interfaces::msg::DrawDeclined>(
        m_lobbyName + "/id" + m_lobbyId + "/DrawDeclined", 10, std::bind(&CheckersPlayerNode::drawDeclinedCallback, this, _1));
    m_drawOfferedSubscription = m_playerNode->create_subscription<turtle_checkers_interfaces::msg::DrawOffered>(
        m_lobbyName + "/id" + m_lobbyId + "/DrawOffered", 10, std::bind(&CheckersPlayerNode::drawOfferedCallback, this, _1));
    m_gameStartSubscription = m_playerNode->create_subscription<turtle_checkers_interfaces::msg::GameStart>(
        m_lobbyName + "/id" + m_lobbyId + "/GameStart", 10, std::bind(&CheckersPlayerNode::gameStartCallback, this, _1));
    m_updateBoardSubscription = m_playerNode->create_subscription<turtle_checkers_interfaces::msg::UpdateBoard>(
        m_lobbyName + "/id" + m_lobbyId + "/UpdateBoard", 10, std::bind(&CheckersPlayerNode::updateBoardCallback, this, _1));
    m_playerJoinedLobbySubscription = m_playerNode->create_subscription<turtle_checkers_interfaces::msg::PlayerJoinedLobby>(
        m_lobbyName + "/id" + m_lobbyId + "/PlayerJoinedLobby", 10, std::bind(&CheckersPlayerNode::playerJoinedLobbyCallback, this, _1));
    m_playerLeftLobbySubscription = m_playerNode->create_subscription<turtle_checkers_interfaces::msg::PlayerLeftLobby>(
        m_lobbyName + "/id" + m_lobbyId + "/PlayerLeftLobby", 10, std::bind(&CheckersPlayerNode::playerLeftLobbyCallback, this, _1));
    m_playerReadiedSubscription = m_playerNode->create_subscription<turtle_checkers_interfaces::msg::PlayerReadied>(
        m_lobbyName + "/id" + m_lobbyId + "/PlayerReadied", 10, std::bind(&CheckersPlayerNode::playerReadiedCallback, this, _1));

    m_forfitPublisher = m_playerNode->create_publisher<turtle_checkers_interfaces::msg::Forfit>(
        m_lobbyName + "/id" + m_lobbyId + "/Forfit", 10);
    m_offerDrawPublisher = m_playerNode->create_publisher<turtle_checkers_interfaces::msg::OfferDraw>(
        m_lobbyName + "/id" + m_lobbyId + "/OfferDraw", 10);
    m_playerReadyPublisher = m_playerNode->create_publisher<turtle_checkers_interfaces::msg::PlayerReady>(
        m_lobbyName + "/id" + m_lobbyId + "/PlayerReady", 10);
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
    if (m_lobbyName != message->lobby_name || m_lobbyId != message->lobby_id)
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
    if (m_lobbyName != message->lobby_name || m_lobbyId != message->lobby_id)
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
    if (m_lobbyName != message->lobby_name || m_lobbyId != message->lobby_id)
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

    if (m_playerName == message->player_name)
    {
        // Do not respond to own offer for a draw
        return;
    }

    m_checkersPlayerWindow->drawOffered();
}

void CheckersPlayerNode::gameStartCallback(const turtle_checkers_interfaces::msg::GameStart::SharedPtr message)
{
    if (m_lobbyName != message->lobby_name || m_lobbyId != message->lobby_id)
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

    TurtlePieceColor playerColor;
    if (message->black_player_name == m_playerName) // Black
    {
        playerColor = TurtlePieceColor::Black;
        RCLCPP_INFO(m_playerNode->get_logger(), "Connected as black player.");
    }
    else if (message->red_player_name == m_playerName) // Red
    {
        playerColor = TurtlePieceColor::Red;
        RCLCPP_INFO(m_playerNode->get_logger(), "Connected as red player.");
    }
    else // Failed to connect (0)
    {
        playerColor = TurtlePieceColor::None;
        RCLCPP_WARN(m_playerNode->get_logger(), "Failed to connect to a game.");
        return;
    }

    GameState gameState = static_cast<GameState>(message->game_state);
    const auto &movableTileIndices = message->movable_tile_indices;
    m_checkersPlayerWindow->gameStarted(gameState, m_lobbyName, m_lobbyId, m_playerName, playerColor, movableTileIndices);

    m_checkersPlayerWindow->update();
}

void CheckersPlayerNode::playerJoinedLobbyCallback(const turtle_checkers_interfaces::msg::PlayerJoinedLobby::SharedPtr message)
{
    if (m_lobbyName != message->lobby_name || m_lobbyId != message->lobby_id)
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
    if (m_lobbyName != message->lobby_name || m_lobbyId != message->lobby_id)
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
    if (m_lobbyName != message->lobby_name || m_lobbyId != message->lobby_id)
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
    if (m_lobbyName != message->lobby_name || m_lobbyId != message->lobby_id)
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

    m_checkersPlayerWindow->updatedBoard(sourceTileIndex, destinationTileIndex, gameState,
                                         slainPieceTileIndex, kingPiece, movableTileIndices);

    m_checkersPlayerWindow->update();
}

void CheckersPlayerNode::requestPieceMove(size_t sourceTileIndex, size_t destinationTileIndex)
{
    auto request = std::make_shared<turtle_checkers_interfaces::srv::RequestPieceMove::Request>();
    request->lobby_name = m_lobbyName;
    request->lobby_id = m_lobbyId;
    request->source_tile_index = sourceTileIndex;
    request->destination_tile_index = destinationTileIndex;
    request->checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::srv::RequestPieceMove::Request::SharedPtr>{}(request),
        m_publicKey, m_privateKey);
    m_requestPieceMoveClient->async_send_request(request,
                                                 std::bind(&CheckersPlayerNode::requestPieceMoveResponse, this, std::placeholders::_1));

    m_checkersPlayerWindow->update();
}

void CheckersPlayerNode::requestReachableTiles(size_t selectedPieceTileIndex)
{
    auto request = std::make_shared<turtle_checkers_interfaces::srv::RequestReachableTiles::Request>();
    request->lobby_name = m_lobbyName;
    request->lobby_id = m_lobbyId;
    request->tile_index = selectedPieceTileIndex;
    request->checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::srv::RequestReachableTiles::Request::SharedPtr>{}(request),
        m_publicKey, m_privateKey);
    m_requestReachableTilesClient->async_send_request(request,
                                                      std::bind(&CheckersPlayerNode::requestReachableTilesResponse, this, std::placeholders::_1));

    m_checkersPlayerWindow->update();
}

void CheckersPlayerNode::offerDraw()
{
    auto message = turtle_checkers_interfaces::msg::OfferDraw();
    message.lobby_name = m_lobbyName;
    message.lobby_id = m_lobbyId;
    message.player_name = m_playerName;
    message.accept_draw = true;
    message.checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::msg::OfferDraw>{}(message),
        m_publicKey, m_privateKey);
    m_offerDrawPublisher->publish(message);
}

void CheckersPlayerNode::declineDraw()
{
    auto message = turtle_checkers_interfaces::msg::OfferDraw();
    message.lobby_name = m_lobbyName;
    message.lobby_id = m_lobbyId;
    message.player_name = m_playerName;
    message.accept_draw = false;
    message.checksum_sig = RSAKeyGenerator::createChecksumSignature(
        std::hash<turtle_checkers_interfaces::msg::OfferDraw>{}(message),
        m_publicKey, m_privateKey);
    m_offerDrawPublisher->publish(message);
}

void CheckersPlayerNode::forfit()
{
    auto message = turtle_checkers_interfaces::msg::Forfit();
    message.lobby_name = m_lobbyName;
    message.lobby_id = m_lobbyId;
    message.player_name = m_playerName;
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