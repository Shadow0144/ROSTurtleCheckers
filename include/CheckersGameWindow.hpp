#pragma once

#ifndef Q_MOC_RUN			  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include "CheckersConsts.hpp" // NO LINT
#include "CheckersBoardRender.hpp"
#include "TileRender.hpp"
#include "TurtlePieceRender.hpp"
#include "TurtleGraveyard.hpp"
#include "HUD.hpp"
#endif

#include <QFrame>
#include <QImage>
#include <QPainter>
#include <QPaintEvent>
#include <QVector>
#include <QTimer>

// This prevents a MOC error with versions of boost >= 1.48
#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "turtle_checkers_interfaces/srv/connect_to_game.hpp"
#include "turtle_checkers_interfaces/srv/request_piece_move.hpp"
#include "turtle_checkers_interfaces/srv/request_reachable_tiles.hpp"
#include "turtle_checkers_interfaces/msg/declare_winner.hpp"
#include "turtle_checkers_interfaces/msg/game_start.hpp"
#include "turtle_checkers_interfaces/msg/player_ready.hpp"
#include "turtle_checkers_interfaces/msg/update_board.hpp"

#include <string>
#include <vector>
#endif

// A single animation frame of the Turtle Checkers game
class CheckersGameWindow : public QFrame
{
	Q_OBJECT
public:
	CheckersGameWindow(
		rclcpp::Node::SharedPtr &nodeHandle,
		const std::string &playerName,
		QWidget *parent = 0,
		Qt::WindowFlags windowFlags = Qt::WindowFlags());
	~CheckersGameWindow();

	void setupGame();

protected:
	void mouseMoveEvent(QMouseEvent *event) override;
	void mousePressEvent(QMouseEvent *event) override;

	void paintEvent(QPaintEvent *event) override;

private slots:
	void onUpdate();

private:
	void connectToGameResponse(rclcpp::Client<turtle_checkers_interfaces::srv::ConnectToGame>::SharedFuture future);
	void requestReachableTilesResponse(rclcpp::Client<turtle_checkers_interfaces::srv::RequestReachableTiles>::SharedFuture future);
	void requestPieceMoveResponse(rclcpp::Client<turtle_checkers_interfaces::srv::RequestPieceMove>::SharedFuture future);

	void declareWinnerCallback(const turtle_checkers_interfaces::msg::DeclareWinner::SharedPtr message);
	void gameStartCallback(const turtle_checkers_interfaces::msg::GameStart::SharedPtr message);
	void updateBoardCallback(const turtle_checkers_interfaces::msg::UpdateBoard::SharedPtr message);

	void parameterEventCallback(const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr event);

	bool isOwnTurn();

	void clearSelections();

	rclcpp::Node::SharedPtr m_nodeHandle;

	std::string m_lobbyName;
	std::string m_playerName;
	TurtlePieceColor m_playerColor;

	GameState m_gameState;

    Winner m_winner;

	CheckersBoardRenderPtr m_board;
	TurtleGraveyardPtr m_blackPlayerGraveyard; // Black player's graveyard containing the slain red pieces
	TurtleGraveyardPtr m_redPlayerGraveyard;   // Red player's graveyard containing the slain black pieces
	HUDPtr m_hud;

    QTimer *m_updateTimer;

	rclcpp::Client<turtle_checkers_interfaces::srv::ConnectToGame>::SharedPtr m_connectToGameClient;
	rclcpp::Client<turtle_checkers_interfaces::srv::RequestPieceMove>::SharedPtr m_requestPieceMoveClient;
	rclcpp::Client<turtle_checkers_interfaces::srv::RequestReachableTiles>::SharedPtr m_requestReachableTilesClient;

	rclcpp::Subscription<turtle_checkers_interfaces::msg::DeclareWinner>::SharedPtr m_declareWinnerSubscription;
	rclcpp::Subscription<turtle_checkers_interfaces::msg::GameStart>::SharedPtr m_gameStartSubscription;
	rclcpp::Subscription<turtle_checkers_interfaces::msg::UpdateBoard>::SharedPtr m_updateBoardSubscription;

	rclcpp::Publisher<turtle_checkers_interfaces::msg::PlayerReady>::SharedPtr m_playerReadyPublisher;
};

typedef std::unique_ptr<CheckersGameWindow> CheckersGameWindowUniPtr;
typedef std::shared_ptr<CheckersGameWindow> CheckersGameWindowShrPtr;