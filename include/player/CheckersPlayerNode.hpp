#pragma once

#include "rclcpp/rclcpp.hpp"

#include "turtle_checkers_interfaces/msg/chat_message.hpp"
#include "turtle_checkers_interfaces/msg/declare_winner.hpp"
#include "turtle_checkers_interfaces/msg/draw_declined.hpp"
#include "turtle_checkers_interfaces/msg/draw_offered.hpp"
#include "turtle_checkers_interfaces/msg/force_logout_account.hpp"
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
#include "turtle_checkers_interfaces/msg/report_player.hpp"
#include "turtle_checkers_interfaces/msg/timer_changed.hpp"
#include "turtle_checkers_interfaces/msg/update_board.hpp"
#include "turtle_checkers_interfaces/msg/update_chat.hpp"
#include "turtle_checkers_interfaces/msg/update_lobby_owner.hpp"
#include "turtle_checkers_interfaces/msg/update_timer.hpp"
#include "turtle_checkers_interfaces/srv/connect_to_game_master.hpp"
#include "turtle_checkers_interfaces/srv/create_account.hpp"
#include "turtle_checkers_interfaces/srv/create_lobby.hpp"
#include "turtle_checkers_interfaces/srv/get_lobby_list.hpp"
#include "turtle_checkers_interfaces/srv/get_statistics.hpp"
#include "turtle_checkers_interfaces/srv/join_lobby.hpp"
#include "turtle_checkers_interfaces/srv/log_in_account.hpp"
#include "turtle_checkers_interfaces/srv/request_piece_move.hpp"
#include "turtle_checkers_interfaces/srv/request_reachable_tiles.hpp"
#include "turtle_checkers_interfaces/srv/resync_board.hpp"

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

    void createAccount(const std::string &playerName,
                       const std::string &playerPassword);
    void logInAccount(const std::string &playerName,
                      const std::string &playerPassword);
    void logOutAccount();

    void requestStatistics(const std::string &playerName);

    void createLobby(const std::string &playerName,
                     const std::string &lobbyName,
                     const std::string &lobbyPassword,
                     TurtlePieceColor playerDesiredColor);
    void joinLobby(const std::string &playerName,
                   const std::string &lobbyName,
                   const std::string &lobbyId,
                   const std::string &lobbyPassword,
                   TurtlePieceColor playerDesiredColor);
    void getLobbyList();
    void leaveLobby();
    void kickPlayer(const std::string &playerName);
    void setReady(bool ready);
    void setTimer(uint64_t timerSeconds);

    void reportPlayer(const std::string &chatMessages);

    void sendChatMessage(const std::string &chatMessage);
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
    void declareWinnerCallback(const turtle_checkers_interfaces::msg::DeclareWinner::SharedPtr message);
    void drawDeclinedCallback(const turtle_checkers_interfaces::msg::DrawDeclined::SharedPtr message);
    void drawOfferedCallback(const turtle_checkers_interfaces::msg::DrawOffered::SharedPtr message);
    void forceLogoutAccountCallback(const turtle_checkers_interfaces::msg::ForceLogoutAccount::SharedPtr message);
    void gameStartCallback(const turtle_checkers_interfaces::msg::GameStart::SharedPtr message);
    void updateBoardCallback(const turtle_checkers_interfaces::msg::UpdateBoard::SharedPtr message);
    void playerJoinedLobbyCallback(const turtle_checkers_interfaces::msg::PlayerJoinedLobby::SharedPtr message);
    void playerLeftLobbyCallback(const turtle_checkers_interfaces::msg::PlayerLeftLobby::SharedPtr message);
    void playerReadiedCallback(const turtle_checkers_interfaces::msg::PlayerReadied::SharedPtr message);
    void updateChatCallback(const turtle_checkers_interfaces::msg::UpdateChat::SharedPtr message);
    void updateLobbyOwnerCallback(const turtle_checkers_interfaces::msg::UpdateLobbyOwner::SharedPtr message);
    void updateTimerCallback(const turtle_checkers_interfaces::msg::UpdateTimer::SharedPtr message);

    void connectToGameMasterResponse(rclcpp::Client<turtle_checkers_interfaces::srv::ConnectToGameMaster>::SharedFuture future);
    void createAccountResponse(rclcpp::Client<turtle_checkers_interfaces::srv::CreateAccount>::SharedFuture future);
    void createLobbyResponse(rclcpp::Client<turtle_checkers_interfaces::srv::CreateLobby>::SharedFuture future);
    void getLobbyListResponse(rclcpp::Client<turtle_checkers_interfaces::srv::GetLobbyList>::SharedFuture future);
    void getStatisticsResponse(rclcpp::Client<turtle_checkers_interfaces::srv::GetStatistics>::SharedFuture future);
    void joinLobbyResponse(rclcpp::Client<turtle_checkers_interfaces::srv::JoinLobby>::SharedFuture future);
    void logInAccountResponse(rclcpp::Client<turtle_checkers_interfaces::srv::LogInAccount>::SharedFuture future);
    void requestReachableTilesResponse(rclcpp::Client<turtle_checkers_interfaces::srv::RequestReachableTiles>::SharedFuture future);
    void requestPieceMoveResponse(rclcpp::Client<turtle_checkers_interfaces::srv::RequestPieceMove>::SharedFuture future);
    void resyncBoardResponse(rclcpp::Client<turtle_checkers_interfaces::srv::ResyncBoard>::SharedFuture future);

    void parameterEventCallback(const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr event);

    void connectToGameMaster();

    void createLobbyInterfaces(const std::string &lobbyName, const std::string &lobbyId);

    std::shared_ptr<rclcpp::Node> m_playerNode;

    std::unique_ptr<CheckersPlayerWindow> m_checkersPlayerWindow;

    rclcpp::Publisher<turtle_checkers_interfaces::msg::ChatMessage>::SharedPtr m_chatMessagePublisher;
    rclcpp::Publisher<turtle_checkers_interfaces::msg::Forfit>::SharedPtr m_forfitPublisher;
    rclcpp::Publisher<turtle_checkers_interfaces::msg::KickPlayer>::SharedPtr m_kickPlayerPublisher;
    rclcpp::Publisher<turtle_checkers_interfaces::msg::LeaveLobby>::SharedPtr m_leaveLobbyPublisher;
    rclcpp::Publisher<turtle_checkers_interfaces::msg::LogOutAccount>::SharedPtr m_logOutAccountPublisher;
    rclcpp::Publisher<turtle_checkers_interfaces::msg::OfferDraw>::SharedPtr m_offerDrawPublisher;
    rclcpp::Publisher<turtle_checkers_interfaces::msg::PlayerReady>::SharedPtr m_playerReadyPublisher;
    rclcpp::Publisher<turtle_checkers_interfaces::msg::ReportPlayer>::SharedPtr m_reportPlayerPublisher;
    rclcpp::Publisher<turtle_checkers_interfaces::msg::TimerChanged>::SharedPtr m_timerChangedPublisher;

    rclcpp::Subscription<turtle_checkers_interfaces::msg::DeclareWinner>::SharedPtr m_declareWinnerSubscription;
    rclcpp::Subscription<turtle_checkers_interfaces::msg::DrawDeclined>::SharedPtr m_drawDeclinedSubscription;
    rclcpp::Subscription<turtle_checkers_interfaces::msg::DrawOffered>::SharedPtr m_drawOfferedSubscription;
    rclcpp::Subscription<turtle_checkers_interfaces::msg::ForceLogoutAccount>::SharedPtr m_forceLogoutAccountSubscription;
    rclcpp::Subscription<turtle_checkers_interfaces::msg::GameStart>::SharedPtr m_gameStartSubscription;
    rclcpp::Subscription<turtle_checkers_interfaces::msg::UpdateBoard>::SharedPtr m_updateBoardSubscription;
    rclcpp::Subscription<turtle_checkers_interfaces::msg::PlayerJoinedLobby>::SharedPtr m_playerJoinedLobbySubscription;
    rclcpp::Subscription<turtle_checkers_interfaces::msg::PlayerLeftLobby>::SharedPtr m_playerLeftLobbySubscription;
    rclcpp::Subscription<turtle_checkers_interfaces::msg::PlayerReadied>::SharedPtr m_playerReadiedSubscription;
    rclcpp::Subscription<turtle_checkers_interfaces::msg::UpdateChat>::SharedPtr m_updateChatSubscription;
    rclcpp::Subscription<turtle_checkers_interfaces::msg::UpdateLobbyOwner>::SharedPtr m_updateLobbyOwnerSubscription;
    rclcpp::Subscription<turtle_checkers_interfaces::msg::UpdateTimer>::SharedPtr m_updateTimerSubscription;

    rclcpp::Client<turtle_checkers_interfaces::srv::ConnectToGameMaster>::SharedPtr m_connectToGameMasterClient;
    rclcpp::Client<turtle_checkers_interfaces::srv::CreateAccount>::SharedPtr m_createAccountClient;
    rclcpp::Client<turtle_checkers_interfaces::srv::LogInAccount>::SharedPtr m_logInAccountClient;
    rclcpp::Client<turtle_checkers_interfaces::srv::CreateLobby>::SharedPtr m_createLobbyClient;
    rclcpp::Client<turtle_checkers_interfaces::srv::GetLobbyList>::SharedPtr m_getLobbyListClient;
    rclcpp::Client<turtle_checkers_interfaces::srv::GetStatistics>::SharedPtr m_getStatisticsClient;
    rclcpp::Client<turtle_checkers_interfaces::srv::JoinLobby>::SharedPtr m_joinLobbyClient;
    rclcpp::Client<turtle_checkers_interfaces::srv::RequestPieceMove>::SharedPtr m_requestPieceMoveClient;
    rclcpp::Client<turtle_checkers_interfaces::srv::RequestReachableTiles>::SharedPtr m_requestReachableTilesClient;
    rclcpp::Client<turtle_checkers_interfaces::srv::ResyncBoard>::SharedPtr m_resyncBoardClient;

    uint64_t m_privateKey;
    uint64_t m_publicKey;
    uint64_t m_gameMasterPublicKey;

    QTimer *m_updateTimer;
};

typedef std::weak_ptr<CheckersPlayerNode> CheckersPlayerNodeWkPtr;