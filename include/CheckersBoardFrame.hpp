#pragma once

#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include "CheckersConsts.hpp" // NO LINT
#include "TileRender.hpp"
#include "TurtlePiece.hpp"
#include "TurtleGraveyard.hpp"
#include "HUD.hpp"
#endif

#include <QFrame>
#include <QImage>
#include <QPainter>
#include <QPaintEvent>
#include <QTimer>
#include <QVector>

// This prevents a MOC error with versions of boost >= 1.48
#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "turtle_checkers_interfaces/srv/connect_to_game.hpp"
#include "turtle_checkers_interfaces/srv/request_reachable_tiles.hpp"
#include "turtle_checkers_interfaces/srv/request_piece_move.hpp"
#include "turtle_checkers_interfaces/msg/update_board.hpp"
#include "turtle_checkers_interfaces/msg/update_game_state.hpp"

#include <map>
#include <string>
#endif

// A single animation frame of the Turtle Checkers game
class CheckersBoardFrame : public QFrame
{
	Q_OBJECT
public:
	CheckersBoardFrame(
		rclcpp::Node::SharedPtr &nodeHandle,
		const std::string &playerName,
		QWidget *parent = 0,
		Qt::WindowFlags windowFlags = Qt::WindowFlags());
	~CheckersBoardFrame();

	void setupGame();

protected:
	void mouseMoveEvent(QMouseEvent *event) override;
	void mousePressEvent(QMouseEvent *event) override;

	void paintEvent(QPaintEvent *event);

private slots:
	void onUpdate();

private:
	void clear();

	void spawnTiles();

	void clearPieces();
	void spawnPieces();
	void spawnTurtle(const std::string &name, bool black, float x, float y, float angle, size_t imageIndex);

	void connectToGameResponse(rclcpp::Client<turtle_checkers_interfaces::srv::ConnectToGame>::SharedFuture future);
	void requestReachableTilesResponse(rclcpp::Client<turtle_checkers_interfaces::srv::RequestReachableTiles>::SharedFuture future);
	void requestPieceMoveResponse(rclcpp::Client<turtle_checkers_interfaces::srv::RequestPieceMove>::SharedFuture future);

	void updateGameStateCallback(const turtle_checkers_interfaces::msg::UpdateGameState::SharedPtr message);
	void updateBoardCallback(const turtle_checkers_interfaces::msg::UpdateBoard::SharedPtr message);

	void parameterEventCallback(const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr);

	void handleMouseMove(QMouseEvent *event);
	void handleMouseClick(QMouseEvent *event);

	void clearSelections();

	rclcpp::Node::SharedPtr m_nodeHandle;

	std::string m_playerName;
	TurtlePieceColor m_playerColor;

	GameState m_gameState;

	std::string m_selectedPieceName;
	std::size_t m_sourceTileIndex;
	std::size_t m_destinationTileIndex;
	bool m_moveSelected;

	rclcpp::Client<turtle_checkers_interfaces::srv::ConnectToGame>::SharedPtr m_connectToGameClient;
	rclcpp::Client<turtle_checkers_interfaces::srv::RequestReachableTiles>::SharedPtr m_requestReachableTilesClient;
	rclcpp::Client<turtle_checkers_interfaces::srv::RequestPieceMove>::SharedPtr m_requestPieceMoveClient;

	rclcpp::Subscription<turtle_checkers_interfaces::msg::UpdateGameState>::SharedPtr m_updateGameStateSubscription;
	rclcpp::Subscription<turtle_checkers_interfaces::msg::UpdateBoard>::SharedPtr m_updateBoardSubscription;

	typedef std::map<std::string, TurtlePiecePtr> TurtlePiecesMap;
	TurtlePiecesMap m_blackTurtles;
	TurtlePiecesMap m_redTurtles;

	QVector<QImage> m_blackTurtleImages;
	size_t m_blackImageIndex = 0u;
	QVector<QImage> m_redTurtleImages;
	size_t m_redImageIndex = 0u;
	QVector<QImage> m_kingTurtleImages;
	QVector<QImage> m_highlightTurtleImages;
	QVector<QImage> m_selectTurtleImages;
	QVector<QImage> m_deadTurtleImages;

	TileRenderPtr m_tileRenders[NUM_PLAYABLE_TILES];
	int m_highlightedTile = -1; // No tile is highlighted

	TurtleGraveyardPtr m_blackPlayerGraveyard; // Black player's graveyard containing the slain red pieces
	TurtleGraveyardPtr m_redPlayerGraveyard; // Red player's graveyard containing the slain black pieces

	HUDPtr m_hud;

	size_t m_blackTurtlesRemaining;
	size_t m_redTurtlesRemaining;
	
	QTimer *m_updateTimer;
};

typedef std::unique_ptr<CheckersBoardFrame> CheckersBoardFrameUniPtr;
typedef std::shared_ptr<CheckersBoardFrame> CheckersBoardFrameShrPtr;