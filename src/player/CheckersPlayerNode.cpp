#include "player/CheckersPlayerNode.hpp"

// This prevents a MOC error with versions of boost >= 1.48
// #ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
// #endif

#include <rclcpp/rclcpp.hpp>

#include "turtle_checkers_interfaces/srv/connect_to_game.hpp"
#include "turtle_checkers_interfaces/srv/request_piece_move.hpp"
#include "turtle_checkers_interfaces/srv/request_reachable_tiles.hpp"
#include "turtle_checkers_interfaces/msg/declare_winner.hpp"
#include "turtle_checkers_interfaces/msg/game_start.hpp"
#include "turtle_checkers_interfaces/msg/player_ready.hpp"
#include "turtle_checkers_interfaces/msg/update_board.hpp"

#include <QApplication>
#include <QTimer>

#include <algorithm>

#include "shared/RSAKeyGenerator.hpp"
#include "player/CheckersPlayerWindow.hpp"

using std::placeholders::_1;

// Parses command line options
char *getCmdOption(char **begin, char **end, const std::string &option)
{
    char **itr = std::find(begin, end, option);
    if (itr != end && ++itr != end)
    {
        return *itr;
    }
    return 0;
}

CheckersPlayerNode::CheckersPlayerNode(int &argc, char **argv)
    : QApplication(argc, argv)
{
    m_playerName = getCmdOption(argv, argv + argc, "-p");
    if (m_playerName.empty())
    {
        m_playerName = getCmdOption(argv, argv + argc, "-player_name");
    }

    rclcpp::init(argc, argv);
    m_playerNode = rclcpp::Node::make_shared("checkers_player_node");

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
    if (m_playerName.empty())
    {
        RCLCPP_ERROR(m_playerNode->get_logger(), "You need to provide a player name with -p or -player_name");
        return -1;
    }
    
    m_checkersApp = std::make_unique<CheckersPlayerWindow>(weak_from_this(), m_playerName);

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

    m_connectToGameClient = m_playerNode->create_client<turtle_checkers_interfaces::srv::ConnectToGame>("ConnectToGame");
    while (!m_connectToGameClient->wait_for_service(std::chrono::seconds(10)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }
    auto request = std::make_shared<turtle_checkers_interfaces::srv::ConnectToGame::Request>();
    request->player_name = m_playerName;
    m_connectToGameClient->async_send_request(request,
                                              std::bind(&CheckersPlayerNode::connectToGameResponse,
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
        if (m_checkersApp)
        {
            m_checkersApp->update();
        }
    }
}

void CheckersPlayerNode::connectToGameResponse(rclcpp::Client<turtle_checkers_interfaces::srv::ConnectToGame>::SharedFuture future)
{
    auto result = future.get();

    m_lobbyPublicKey = result->game_master_public_key;

    auto lobbyName = result->lobby_name;
    TurtlePieceColor playerColor;
    if (result->player_color == 1) // Black
    {
        playerColor = TurtlePieceColor::Black;
        RCLCPP_INFO(m_playerNode->get_logger(), "Connected as black player.");
    }
    else if (result->player_color == 2) // Red
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

    m_checkersApp->connectedToGame(lobbyName, playerColor);
    m_checkersApp->show();

    // Create the services, subscriptions, and publishers now that we have the full topic names
    m_requestPieceMoveClient = m_playerNode->create_client<turtle_checkers_interfaces::srv::RequestPieceMove>(
        lobbyName + "/RequestPieceMove");
    m_requestReachableTilesClient = m_playerNode->create_client<turtle_checkers_interfaces::srv::RequestReachableTiles>(
        lobbyName + "/RequestReachableTiles");

    m_declareWinnerSubscription = m_playerNode->create_subscription<turtle_checkers_interfaces::msg::DeclareWinner>(
        lobbyName + "/DeclareWinner", 10, std::bind(&CheckersPlayerNode::declareWinnerCallback, this, _1));
    m_gameStartSubscription = m_playerNode->create_subscription<turtle_checkers_interfaces::msg::GameStart>(
        lobbyName + "/GameStart", 10, std::bind(&CheckersPlayerNode::gameStartCallback, this, _1));
    m_updateBoardSubscription = m_playerNode->create_subscription<turtle_checkers_interfaces::msg::UpdateBoard>(
        lobbyName + "/UpdateBoard", 10, std::bind(&CheckersPlayerNode::updateBoardCallback, this, _1));

    m_playerReadyPublisher = m_playerNode->create_publisher<turtle_checkers_interfaces::msg::PlayerReady>(
        lobbyName + "/PlayerReady", 10);

    // Tell the game master this player is ready
    auto message = turtle_checkers_interfaces::msg::PlayerReady();
    message.lobby_name = lobbyName;
    message.player_name = m_playerName;
    m_playerReadyPublisher->publish(message);

    m_checkersApp->update();
}

void CheckersPlayerNode::requestPieceMoveResponse(rclcpp::Client<turtle_checkers_interfaces::srv::RequestPieceMove>::SharedFuture future)
{
    auto result = future.get();
    m_checkersApp->requestedPieceMoveAccepted(result->move_accepted);

    m_checkersApp->update();
}

void CheckersPlayerNode::requestReachableTilesResponse(rclcpp::Client<turtle_checkers_interfaces::srv::RequestReachableTiles>::SharedFuture future)
{
    auto result = future.get();
    const auto &reachableTileIndices = result->reachable_tile_indices;
    m_checkersApp->requestedReachableTiles(reachableTileIndices);

    m_checkersApp->update();
}

void CheckersPlayerNode::declareWinnerCallback(const turtle_checkers_interfaces::msg::DeclareWinner::SharedPtr message)
{
    if (m_checkersApp->getLobbyName() != message->lobby_name)
    {
        return;
    }

    Winner winner = static_cast<Winner>(message->winner);
    m_checkersApp->declaredWinner(winner);

    m_checkersApp->update();
}

void CheckersPlayerNode::gameStartCallback(const turtle_checkers_interfaces::msg::GameStart::SharedPtr message)
{
    if (m_checkersApp->getLobbyName() != message->lobby_name)
    {
        return;
    }

    GameState gameState = static_cast<GameState>(message->game_state);
    const auto &movableTileIndices = message->movable_tile_indices;
    m_checkersApp->gameStarted(gameState, movableTileIndices);

    m_checkersApp->update();
}

void CheckersPlayerNode::updateBoardCallback(const turtle_checkers_interfaces::msg::UpdateBoard::SharedPtr message)
{
    if (m_checkersApp->getLobbyName() != message->lobby_name)
    {
        return;
    }

    auto sourceTileIndex = message->source_tile_index;
    auto destinationTileIndex = message->destination_tile_index;
    auto gameState = static_cast<GameState>(message->game_state);
    auto slainPieceTileIndex = message->slain_piece_tile_index;
    auto kingPiece = message->king_piece;
    const auto &movableTileIndices = message->movable_tile_indices;

    m_checkersApp->updatedBoard(sourceTileIndex, destinationTileIndex, gameState,
                                slainPieceTileIndex, kingPiece, movableTileIndices);

    m_checkersApp->update();
}

void CheckersPlayerNode::requestPieceMove(size_t sourceTileIndex, size_t destinationTileIndex)
{
    auto request = std::make_shared<turtle_checkers_interfaces::srv::RequestPieceMove::Request>();
    request->lobby_name = m_checkersApp->getLobbyName();
    request->source_tile_index = sourceTileIndex;
    request->destination_tile_index = destinationTileIndex;
    m_requestPieceMoveClient->async_send_request(request,
                                                 std::bind(&CheckersPlayerNode::requestPieceMoveResponse, this, std::placeholders::_1));

    m_checkersApp->update();
}

void CheckersPlayerNode::requestReachableTiles(size_t selectedPieceTileIndex)
{
    auto request = std::make_shared<turtle_checkers_interfaces::srv::RequestReachableTiles::Request>();
    request->lobby_name = m_checkersApp->getLobbyName();
    request->tile_index = selectedPieceTileIndex;
    m_requestReachableTilesClient->async_send_request(request,
                                                      std::bind(&CheckersPlayerNode::requestReachableTilesResponse, this, std::placeholders::_1));

    m_checkersApp->update();
}

void CheckersPlayerNode::onUpdate()
{
    if (!rclcpp::ok())
    {
        quit();
        return;
    }

    // m_checkersApp->update();
    rclcpp::spin_some(m_playerNode);
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