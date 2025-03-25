#pragma once

#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include "CheckersConsts.hpp"
#include "TileRender.hpp"
#include "TurtlePiece.hpp" // NO LINT
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

#include <rcl_interfaces/msg/parameter_event.hpp>
#include <std_srvs/srv/empty.hpp>

#include "turtle_checkers_interfaces/srv/request_reachable_tiles.hpp"

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
		TurtlePieceColor playerColor,
		GameState gameState,
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

	void requestReachableTilesResponse(rclcpp::Client<turtle_checkers_interfaces::srv::RequestReachableTiles>::SharedFuture future);

	void parameterEventCallback(const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr);

	rclcpp::Node::SharedPtr m_nodeHandle;

	QTimer *m_updateTimer;

	TurtlePieceColor m_playerColor;

	GameState m_gameState;

	std::string m_selectedPieceName;

	rclcpp::Client<turtle_checkers_interfaces::srv::RequestReachableTiles>::SharedPtr m_requestReachableTilesClient;

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

	TileRenderPtr m_tileRenders[NUM_PLAYABLE_TILES];
	int m_highlightedTile = -1; // No tile is highlighted
};

typedef std::unique_ptr<CheckersBoardFrame> CheckersBoardFrameUniPtr;
typedef std::shared_ptr<CheckersBoardFrame> CheckersBoardFrameShrPtr;