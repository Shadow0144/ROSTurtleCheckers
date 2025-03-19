#pragma once

#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include "turtle_piece.hpp" // NO LINT
#endif

#include <QFrame>
#include <QImage>
#include <QPainter>
#include <QPaintEvent>
#include <QTimer>
#include <QVector>

// This prevents a MOC error with versions of boost >= 1.48
#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <map>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rcl_interfaces/msg/parameter_event.hpp>
#include <std_srvs/srv/empty.hpp>
// #include <turtlesim_msgs/srv/spawn.hpp>
// #include <turtlesim_msgs/srv/kill.hpp>
#endif

// A single animation frame of the Turtle Checkers game
class CheckersBoardFrame : public QFrame
{
	Q_OBJECT
public:
	CheckersBoardFrame(
		rclcpp::Node::SharedPtr &node_handle, QWidget *parent = 0,
		Qt::WindowFlags f = Qt::WindowFlags());
	~CheckersBoardFrame();

	void setupGame();

	std::string spawnTurtle(const std::string &name, bool black, float x, float y, float angle, size_t image_index);

protected:
	void paintEvent(QPaintEvent *event);

private slots:
	void onUpdate();

private:
	void clear();

	void clearPieces();
	void spawnPieces();

	bool clearCallback(
		const std_srvs::srv::Empty::Request::SharedPtr,
		std_srvs::srv::Empty::Response::SharedPtr);
	bool resetCallback(
		const std_srvs::srv::Empty::Request::SharedPtr,
		std_srvs::srv::Empty::Response::SharedPtr);

	void parameterEventCallback(const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr);

	rclcpp::Node::SharedPtr nh_;

	QTimer *update_timer_;
	QImage path_image_;
	QPainter path_painter_;

	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_srv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
	rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

	typedef std::map<std::string, TurtlePiecePtr> TurtlePiecesMap;
	TurtlePiecesMap black_turtles_;
	TurtlePiecesMap red_turtles_;

	QVector<QImage> black_turtle_images_;
	size_t black_image_index = 0u;
	QVector<QImage> red_turtle_images_;
	size_t red_image_index = 0u;

	constexpr static size_t NUM_PLAYABLE_TILES = 32u;
	float tile_centers_x[NUM_PLAYABLE_TILES];
	float tile_centers_y[NUM_PLAYABLE_TILES];
};