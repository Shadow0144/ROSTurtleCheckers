#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "turtle_checkers_interfaces/srv/connect_to_game.hpp"
#include "turtle_checkers_interfaces/srv/request_piece_move.hpp"
#include "turtle_checkers_interfaces/srv/request_reachable_tiles.hpp"
#include "turtle_checkers_interfaces/msg/declare_winner.hpp"
#include "turtle_checkers_interfaces/msg/game_start.hpp"
#include "turtle_checkers_interfaces/msg/player_ready.hpp"
#include "turtle_checkers_interfaces/msg/update_board.hpp"

#include <QApplication>
#include <QTimer>

#include "CheckersConsts.hpp"
#include "TurtlePiece.hpp"

class CheckersPlayerWindow;

class CheckersPlayerNode : public QApplication, public std::enable_shared_from_this<CheckersPlayerNode>
{
    Q_OBJECT
public:
    explicit CheckersPlayerNode(int &argc, char **argv);
    ~CheckersPlayerNode();

    int exec();

    void requestPieceMove(size_t sourceTileIndex, size_t destinationTileIndex);
    void requestReachableTiles(size_t selectedPieceTileIndex);

    std::shared_ptr<rclcpp::Node> &getNodeHandle();

public slots:
    void onUpdate();

private:
    void connectToGameResponse(rclcpp::Client<turtle_checkers_interfaces::srv::ConnectToGame>::SharedFuture future);
    void requestReachableTilesResponse(rclcpp::Client<turtle_checkers_interfaces::srv::RequestReachableTiles>::SharedFuture future);
    void requestPieceMoveResponse(rclcpp::Client<turtle_checkers_interfaces::srv::RequestPieceMove>::SharedFuture future);

    void declareWinnerCallback(const turtle_checkers_interfaces::msg::DeclareWinner::SharedPtr message);
    void gameStartCallback(const turtle_checkers_interfaces::msg::GameStart::SharedPtr message);
    void updateBoardCallback(const turtle_checkers_interfaces::msg::UpdateBoard::SharedPtr message);

    void parameterEventCallback(const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr event);

    std::shared_ptr<rclcpp::Node> m_playerNode;

    std::unique_ptr<CheckersPlayerWindow> m_checkersApp;

    std::string m_playerName;

    rclcpp::Client<turtle_checkers_interfaces::srv::ConnectToGame>::SharedPtr m_connectToGameClient;
    rclcpp::Client<turtle_checkers_interfaces::srv::RequestPieceMove>::SharedPtr m_requestPieceMoveClient;
    rclcpp::Client<turtle_checkers_interfaces::srv::RequestReachableTiles>::SharedPtr m_requestReachableTilesClient;

    rclcpp::Subscription<turtle_checkers_interfaces::msg::DeclareWinner>::SharedPtr m_declareWinnerSubscription;
    rclcpp::Subscription<turtle_checkers_interfaces::msg::GameStart>::SharedPtr m_gameStartSubscription;
    rclcpp::Subscription<turtle_checkers_interfaces::msg::UpdateBoard>::SharedPtr m_updateBoardSubscription;

    rclcpp::Publisher<turtle_checkers_interfaces::msg::PlayerReady>::SharedPtr m_playerReadyPublisher;

    long long m_privateKey;
    long long m_publicKey;
    long long m_lobbyPublicKey;

    QTimer *m_updateTimer;
};

typedef std::weak_ptr<CheckersPlayerNode> CheckersPlayerNodeWkPtr;