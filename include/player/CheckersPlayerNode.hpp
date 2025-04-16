#pragma once

#include "rclcpp/rclcpp.hpp"

#include "turtle_checkers_interfaces/srv/connect_to_game_master.hpp"
#include "turtle_checkers_interfaces/srv/create_lobby.hpp"
#include "turtle_checkers_interfaces/srv/get_lobby_list.hpp"
#include "turtle_checkers_interfaces/srv/join_lobby.hpp"
#include "turtle_checkers_interfaces/srv/request_piece_move.hpp"
#include "turtle_checkers_interfaces/srv/request_reachable_tiles.hpp"
#include "turtle_checkers_interfaces/msg/declare_winner.hpp"
#include "turtle_checkers_interfaces/msg/draw_offered.hpp"
#include "turtle_checkers_interfaces/msg/forfit.hpp"
#include "turtle_checkers_interfaces/msg/game_start.hpp"
#include "turtle_checkers_interfaces/msg/leave_lobby.hpp"
#include "turtle_checkers_interfaces/msg/offer_draw.hpp"
#include "turtle_checkers_interfaces/msg/player_joined_lobby.hpp"
#include "turtle_checkers_interfaces/msg/player_left_lobby.hpp"
#include "turtle_checkers_interfaces/msg/player_ready.hpp"
#include "turtle_checkers_interfaces/msg/update_board.hpp"

#include <QApplication>
#include <QTimer>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "shared/CheckersConsts.hpp"

class CheckersPlayerWindow;

class CheckersPlayerNode : public QApplication, public std::enable_shared_from_this<CheckersPlayerNode>
{
    Q_OBJECT
public:
    explicit CheckersPlayerNode(int &argc, char **argv);
    ~CheckersPlayerNode();

    int exec();

    void createLobby(const std::string &playerName,
                     const std::string &lobbyName,
                     TurtlePieceColor playerDesiredColor);
    void joinLobby(const std::string &playerName,
                   const std::string &lobbyName,
                   const std::string &lobbyId,
                   TurtlePieceColor playerDesiredColor);
    void getLobbyList();
    void leaveLobby();
    void setReady(bool ready);

    void requestPieceMove(size_t sourceTileIndex, size_t destinationTileIndex);
    void requestReachableTiles(size_t selectedPieceTileIndex);

    void offerDraw();
    void declineDraw();
    void forfit();

    std::shared_ptr<rclcpp::Node> &getNodeHandle();

    void shutdown();

public slots:
    void onUpdate();

private:
    void connectToGameMasterResponse(rclcpp::Client<turtle_checkers_interfaces::srv::ConnectToGameMaster>::SharedFuture future);
    void createLobbyResponse(rclcpp::Client<turtle_checkers_interfaces::srv::CreateLobby>::SharedFuture future);
    void getLobbyListResponse(rclcpp::Client<turtle_checkers_interfaces::srv::GetLobbyList>::SharedFuture future);
    void joinLobbyResponse(rclcpp::Client<turtle_checkers_interfaces::srv::JoinLobby>::SharedFuture future);
    void requestReachableTilesResponse(rclcpp::Client<turtle_checkers_interfaces::srv::RequestReachableTiles>::SharedFuture future);
    void requestPieceMoveResponse(rclcpp::Client<turtle_checkers_interfaces::srv::RequestPieceMove>::SharedFuture future);

    void declareWinnerCallback(const turtle_checkers_interfaces::msg::DeclareWinner::SharedPtr message);
    void declareWinnerCallback(const turtle_checkers_interfaces::msg::DrawOffered::SharedPtr message);
    void gameStartCallback(const turtle_checkers_interfaces::msg::GameStart::SharedPtr message);
    void updateBoardCallback(const turtle_checkers_interfaces::msg::UpdateBoard::SharedPtr message);
    void playerJoinedLobbyCallback(const turtle_checkers_interfaces::msg::PlayerJoinedLobby::SharedPtr message);
    void playerLeftLobbyCallback(const turtle_checkers_interfaces::msg::PlayerLeftLobby::SharedPtr message);
    void playerReadyCallback(const turtle_checkers_interfaces::msg::PlayerReady::SharedPtr message);

    void parameterEventCallback(const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr event);

    void createLobbyInterfaces(const std::string &lobbyName, const std::string &lobbyId);

    std::shared_ptr<rclcpp::Node> m_playerNode;

    std::unique_ptr<CheckersPlayerWindow> m_checkersApp;

    std::string m_playerName;
    std::string m_lobbyName;
    std::string m_lobbyId;

    rclcpp::Client<turtle_checkers_interfaces::srv::ConnectToGameMaster>::SharedPtr m_connectToGameMasterClient;
    rclcpp::Client<turtle_checkers_interfaces::srv::CreateLobby>::SharedPtr m_createLobbyClient;
    rclcpp::Client<turtle_checkers_interfaces::srv::GetLobbyList>::SharedPtr m_getLobbyListClient;
    rclcpp::Client<turtle_checkers_interfaces::srv::JoinLobby>::SharedPtr m_joinLobbyClient;
    rclcpp::Client<turtle_checkers_interfaces::srv::RequestPieceMove>::SharedPtr m_requestPieceMoveClient;
    rclcpp::Client<turtle_checkers_interfaces::srv::RequestReachableTiles>::SharedPtr m_requestReachableTilesClient;

    rclcpp::Subscription<turtle_checkers_interfaces::msg::DeclareWinner>::SharedPtr m_declareWinnerSubscription;
    rclcpp::Subscription<turtle_checkers_interfaces::msg::DrawOffered>::SharedPtr m_drawOfferedSubscription;
    rclcpp::Subscription<turtle_checkers_interfaces::msg::GameStart>::SharedPtr m_gameStartSubscription;
    rclcpp::Subscription<turtle_checkers_interfaces::msg::UpdateBoard>::SharedPtr m_updateBoardSubscription;
    rclcpp::Subscription<turtle_checkers_interfaces::msg::PlayerJoinedLobby>::SharedPtr m_playerJoinedLobbySubscription;
    rclcpp::Subscription<turtle_checkers_interfaces::msg::PlayerLeftLobby>::SharedPtr m_playerLeftLobbySubscription;
    rclcpp::Subscription<turtle_checkers_interfaces::msg::PlayerReady>::SharedPtr m_playerReadySubscription;

    rclcpp::Publisher<turtle_checkers_interfaces::msg::Forfit>::SharedPtr m_forfitPublisher;
    rclcpp::Publisher<turtle_checkers_interfaces::msg::LeaveLobby>::SharedPtr m_leaveLobbyPublisher;
    rclcpp::Publisher<turtle_checkers_interfaces::msg::OfferDraw>::SharedPtr m_offerDrawPublisher;
    rclcpp::Publisher<turtle_checkers_interfaces::msg::PlayerReady>::SharedPtr m_playerReadyPublisher;

    long long m_privateKey;
    long long m_publicKey;
    long long m_lobbyPublicKey;

    QTimer *m_updateTimer;
};

typedef std::weak_ptr<CheckersPlayerNode> CheckersPlayerNodeWkPtr;