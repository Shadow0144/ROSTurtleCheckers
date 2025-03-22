#pragma once

#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
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
	enum class GameState
	{
		SelectPiece,
		SelectTile,
		WaitingOnOtherPlayer,
		GameFinished
	};

	CheckersBoardFrame(
		rclcpp::Node::SharedPtr &node_handle,
		TurtlePiece::TurtleColor player_color,
		GameState game_state,
		QWidget *parent = 0,
		Qt::WindowFlags f = Qt::WindowFlags());
	~CheckersBoardFrame();

	void setupGame();

protected:
	void mousePressEvent(QMouseEvent *event) override;

	void paintEvent(QPaintEvent *event);

private slots:
	void onUpdate();

private:
	void clear();

	void spawnTiles();

	void clearPieces();
	void spawnPieces();
	std::string spawnTurtle(const std::string &name, bool black, float x, float y, float angle, size_t image_index);

	bool clearCallback(
		const std_srvs::srv::Empty::Request::SharedPtr,
		std_srvs::srv::Empty::Response::SharedPtr);
	bool resetCallback(
		const std_srvs::srv::Empty::Request::SharedPtr,
		std_srvs::srv::Empty::Response::SharedPtr);

	void requestReachableTilesResponse(rclcpp::Client<turtle_checkers_interfaces::srv::RequestReachableTiles>::SharedFuture future);

	void parameterEventCallback(const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr);

	rclcpp::Node::SharedPtr nh_;

	QTimer *update_timer_;
	QImage path_image_;
	QPainter path_painter_;

	TurtlePiece::TurtleColor player_color_;

	GameState game_state_;

	std::string selected_piece_;

	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_srv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
	rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

	rclcpp::Client<turtle_checkers_interfaces::srv::RequestReachableTiles>::SharedPtr requestReachableTilesClient;

	typedef std::map<std::string, TurtlePiecePtr> TurtlePiecesMap;
	TurtlePiecesMap black_turtles_;
	TurtlePiecesMap red_turtles_;

	QVector<QImage> black_turtle_images_;
	size_t black_image_index = 0u;
	QVector<QImage> red_turtle_images_;
	size_t red_image_index = 0u;
	QVector<QImage> highlight_turtle_images_;
	QVector<QImage> king_turtle_images_;

	constexpr static size_t NUM_PLAYABLE_TILES = 32u;
	TileRenderPtr tile_renders[NUM_PLAYABLE_TILES];
	int highlighted_tile = -1; // No tile is highlighted
};

typedef std::unique_ptr<CheckersBoardFrame> CheckersBoardFrameUniPtr;
typedef std::shared_ptr<CheckersBoardFrame> CheckersBoardFrameShrPtr;