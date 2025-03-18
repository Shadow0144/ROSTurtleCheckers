#pragma once

#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
// #include "turtle.hpp" // NO LINT
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

	std::string spawnRedTurtle(const std::string& name, float x, float y, float angle);
	std::string spawnRedTurtle(const std::string& name, float x, float y, float angle, size_t index);
	std::string spawnBlackTurtle(const std::string& name, float x, float y, float angle);
	std::string spawnBlackTurtle(const std::string& name, float x, float y, float angle, size_t index);

protected:
	void paintEvent(QPaintEvent *event);

private slots:
	void onUpdate();

private:
	// void updateTurtles();
	void clear();
	// bool hasTurtle(const std::string &name);

	bool clearCallback(
		const std_srvs::srv::Empty::Request::SharedPtr,
		std_srvs::srv::Empty::Response::SharedPtr);
	bool resetCallback(
		const std_srvs::srv::Empty::Request::SharedPtr,
		std_srvs::srv::Empty::Response::SharedPtr);
	// bool spawnCallback(
	//	const turtlesim_msgs::srv::Spawn::Request::SharedPtr,
	//	turtlesim_msgs::srv::Spawn::Response::SharedPtr);
	// bool killCallback(
	//	const turtlesim_msgs::srv::Kill::Request::SharedPtr,
	//	turtlesim_msgs::srv::Kill::Response::SharedPtr);

	void parameterEventCallback(const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr);

	rclcpp::Node::SharedPtr nh_;

	QTimer *update_timer_;
	QImage path_image_;
	QPainter path_painter_;

	uint64_t frame_count_;

	rclcpp::Time last_turtle_update_;

	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_srv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
	// rclcpp::Service<turtlesim_msgs::srv::Spawn>::SharedPtr spawn_srv_;
	// rclcpp::Service<turtlesim_msgs::srv::Kill>::SharedPtr kill_srv_;
	rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

	// typedef std::map<std::string, TurtlePtr> M_Turtle;
	// M_Turtle turtles_;
	uint32_t id_counter_;

	QVector<QImage> red_turtle_images_;
	QVector<QImage> black_turtle_images_;

	float meter_;
	float width_in_meters_;
	float height_in_meters_;
};