#include "checkers_board_frame.hpp"

#include <QPointF>

#include <cstdlib>
#include <ctime>
#include <functional>
#include <string>
#include <math.h>

#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

// #include "turtlesim_msgs/srv/kill.hpp"
// #include "turtlesim_msgs/srv/spawn.hpp"

// Constants
constexpr size_t NUM_PIECES_PER_PLAYER = 12u;

constexpr size_t NUM_COLS_ROWS = 8u;

constexpr int RED_SQUARES_BG_RGB[3] = {255u, 0u, 0u};
constexpr int BLACK_SQUARES_BG_RGB[3] = {0u, 0u, 0u};

constexpr float RED_ROTATION = 1.5f * M_PI;
constexpr float BLACK_ROTATION = 0.5f * M_PI;

CheckersBoardFrame::CheckersBoardFrame(rclcpp::Node::SharedPtr &node_handle, QWidget *parent, Qt::WindowFlags f)
	: QFrame(parent, f), 
	path_image_(500, 500, QImage::Format_ARGB32), 
	path_painter_(&path_image_)
{
	setFixedSize(500, 500);
	setWindowTitle("TurtleCheckers");

	srand(time(NULL));

	update_timer_ = new QTimer(this);
	update_timer_->setInterval(16);
	update_timer_->start();

	connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));

	nh_ = node_handle;
	rcl_interfaces::msg::IntegerRange range;
	range.from_value = 0;
	range.step = 1;
	range.to_value = 255;
	rcl_interfaces::msg::ParameterDescriptor background_r_descriptor;
	background_r_descriptor.description = "Red channel of the background color";
	background_r_descriptor.integer_range.push_back(range);
	rcl_interfaces::msg::ParameterDescriptor background_g_descriptor;
	background_g_descriptor.description = "Green channel of the background color";
	background_g_descriptor.integer_range.push_back(range);
	rcl_interfaces::msg::ParameterDescriptor background_b_descriptor;
	background_b_descriptor.description = "Blue channel of the background color";
	background_b_descriptor.integer_range.push_back(range);
	nh_->declare_parameter(
		"background_r", rclcpp::ParameterValue(RED_SQUARES_BG_RGB[0]), background_r_descriptor);
	nh_->declare_parameter(
		"background_g", rclcpp::ParameterValue(RED_SQUARES_BG_RGB[1]), background_g_descriptor);
	nh_->declare_parameter(
		"background_b", rclcpp::ParameterValue(RED_SQUARES_BG_RGB[2]), background_b_descriptor);

	rcl_interfaces::msg::ParameterDescriptor holonomic_descriptor;
	holonomic_descriptor.description = "If true, then turtles will be holonomic";
	nh_->declare_parameter("holonomic", rclcpp::ParameterValue(false), holonomic_descriptor);

	QVector<QString> turtles_images;
	/*turtles_images.append("ardent.png");
	turtles_images.append("bouncy.png");
	turtles_images.append("crystal.png");
	turtles_images.append("dashing.png");
	turtles_images.append("eloquent.png");
	turtles_images.append("foxy.png");
	turtles_images.append("galactic.png");
	turtles_images.append("humble.png");
	turtles_images.append("iron.png");
	turtles_images.append("jazzy.png");*/
	turtles_images.append("rolling");

	QString images_path =
		(ament_index_cpp::get_package_share_directory("turtle_checkers") + "/img/").c_str();
	std::cout << images_path.toStdString() << std::endl;
	for (int i = 0; i < turtles_images.size(); ++i)
	{
		QImage r_img;
		r_img.load(images_path + turtles_images[i] + "_red.png");
		red_turtle_images_.append(r_img);
		QImage b_img;
		b_img.load(images_path + turtles_images[i] + "_black.png");
		black_turtle_images_.append(b_img);
	}

	clear();

	clear_srv_ =
		nh_->create_service<std_srvs::srv::Empty>(
			"clear",
			std::bind(&CheckersBoardFrame::clearCallback, this, std::placeholders::_1, std::placeholders::_2));
	reset_srv_ =
		nh_->create_service<std_srvs::srv::Empty>(
			"reset",
			std::bind(&CheckersBoardFrame::resetCallback, this, std::placeholders::_1, std::placeholders::_2));
	/*
	spawn_srv_ =
	  nh_->create_service<turtlesim_msgs::srv::Spawn>(
	  "spawn",
	  std::bind(&CheckersBoardFrame::spawnCallback, this, std::placeholders::_1, std::placeholders::_2));
	kill_srv_ =
	  nh_->create_service<turtlesim_msgs::srv::Kill>(
	  "kill",
	  std::bind(&CheckersBoardFrame::killCallback, this, std::placeholders::_1, std::placeholders::_2));
	*/
	rclcpp::QoS qos(rclcpp::KeepLast(100), rmw_qos_profile_sensor_data);
	parameter_event_sub_ = nh_->create_subscription<rcl_interfaces::msg::ParameterEvent>(
		"/parameter_events", qos,
		std::bind(&CheckersBoardFrame::parameterEventCallback, this, std::placeholders::_1));

	RCLCPP_INFO(
		nh_->get_logger(), "Starting turtle checkers board with node name %s", nh_->get_fully_qualified_name());


	const float meter = red_turtle_images_[0].height(); // Assume we have at least one red image
	const float width_in_meters = (width() - 1) / meter;
	const float height_in_meters = (height() - 1) / meter;
	const float tile_width = width_in_meters / NUM_COLS_ROWS;
	const float tile_height = height_in_meters / NUM_COLS_ROWS;
	const float tile_half_width = tile_width / 2.0f;
	const float tile_half_height = tile_height / 2.0f;

	for (size_t i = 0u; i < NUM_COLS_ROWS; i++)
	{
		if (i % 2u == 0u)
		{
			tile_centers_x[(i * 4) + 0] = (1 * tile_width) + tile_half_width;
			tile_centers_x[(i * 4) + 1] = (3 * tile_width) + tile_half_width;
			tile_centers_x[(i * 4) + 2] = (5 * tile_width) + tile_half_width;
			tile_centers_x[(i * 4) + 3] = (7 * tile_width) + tile_half_width;
		}
		else
		{
			tile_centers_x[(i * 4) + 0] = (0 * tile_width) + tile_half_width;
			tile_centers_x[(i * 4) + 1] = (2 * tile_width) + tile_half_width;
			tile_centers_x[(i * 4) + 2] = (4 * tile_width) + tile_half_width;
			tile_centers_x[(i * 4) + 3] = (6 * tile_width) + tile_half_width;
		}

		tile_centers_y[(i * 4) + 0] = (i * tile_height) + tile_half_height;
		tile_centers_y[(i * 4) + 1] = (i * tile_height) + tile_half_height;
		tile_centers_y[(i * 4) + 2] = (i * tile_height) + tile_half_height;
		tile_centers_y[(i * 4) + 3] = (i * tile_height) + tile_half_height;
	}

	spawnPieces();
}

CheckersBoardFrame::~CheckersBoardFrame()
{
	delete update_timer_;
}

void CheckersBoardFrame::parameterEventCallback(
	const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr event)
{
	// Only consider events from this node
	if (event->node == nh_->get_fully_qualified_name())
	{
		// Since parameter events for this event aren't expected frequently just always call update()
		update();
	}
}

void CheckersBoardFrame::clearPieces()
{
}

void CheckersBoardFrame::spawnPieces()
{
	// Spawn all the checkers pieces
	// Red
	for (size_t i = 0u; i < NUM_PIECES_PER_PLAYER; i++)
	{
		spawnTurtle("Red" + std::to_string(i + 1),
					true,
					tile_centers_x[i],
					tile_centers_y[i],
					red_image_index);
	}
	// Black
	for (size_t i = 0u; i < NUM_PIECES_PER_PLAYER; i++)
	{
		spawnTurtle("Black" + std::to_string(i + 1),
					false,
					tile_centers_x[i + 20],
					tile_centers_y[i + 20],
					black_image_index);
	}
}

std::string CheckersBoardFrame::spawnTurtle(const std::string &name, bool red, float x, float y, size_t image_index)
{
	if (red)
	{
		red_turtles_[name] = std::make_shared<Turtle>(
			nh_, name, red_turtle_images_[static_cast<int>(image_index)],
			QPointF(x, y), RED_ROTATION);
		RCLCPP_INFO(
			nh_->get_logger(), "Spawning red turtle [%s] at x=[%f], y=[%f], theta=[%f]",
			name.c_str(), x, y, RED_ROTATION);
	}
	else // black
	{
		black_turtles_[name] = std::make_shared<Turtle>(
			nh_, name, black_turtle_images_[static_cast<int>(image_index)],
			QPointF(x, y), BLACK_ROTATION);
		RCLCPP_INFO(
			nh_->get_logger(), "Spawning black turtle [%s] at x=[%f], y=[%f], theta=[%f]",
			name.c_str(), x, y, BLACK_ROTATION);
	}
	update();

	return name;
}

void CheckersBoardFrame::clear()
{
	// Make all pixels fully transparent
	path_image_.fill(qRgba(255, 255, 255, 0));
	update();
}

void CheckersBoardFrame::onUpdate()
{
	if (!rclcpp::ok())
	{
		close();
		return;
	}

	rclcpp::spin_some(nh_);

	// updateTurtles();
}

void CheckersBoardFrame::paintEvent(QPaintEvent *event)
{
	(void)event; // NO LINT
	QPainter painter(this);

	float tile_width = width() / NUM_COLS_ROWS;
	float tile_height = height() / NUM_COLS_ROWS;

	for (size_t i = 0u; i < NUM_COLS_ROWS; i++)
	{
		for (size_t j = 0u; j < NUM_COLS_ROWS; j++)
		{
			int r = ((i + j) % 2 == 0) ? RED_SQUARES_BG_RGB[0] : BLACK_SQUARES_BG_RGB[0];
			int g = ((i + j) % 2 == 0) ? RED_SQUARES_BG_RGB[1] : BLACK_SQUARES_BG_RGB[1];
			int b = ((i + j) % 2 == 0) ? RED_SQUARES_BG_RGB[2] : BLACK_SQUARES_BG_RGB[2];
			QRgb background_color = qRgb(r, g, b);
			painter.fillRect(
				tile_width * i,
				tile_height * j,
				tile_width * (i + 1),
				tile_height * (j + 1),
				background_color);
		}
	}

	painter.drawImage(QPoint(0, 0), path_image_);

	M_Turtle::iterator rit = red_turtles_.begin();
	M_Turtle::iterator rend = red_turtles_.end();
	for (; rit != rend; ++rit)
	{
		rit->second->paint(painter);
	}

	M_Turtle::iterator bit = black_turtles_.begin();
	M_Turtle::iterator bend = black_turtles_.end();
	for (; bit != bend; ++bit)
	{
		bit->second->paint(painter);
	}
}

bool CheckersBoardFrame::clearCallback(
	const std_srvs::srv::Empty::Request::SharedPtr,
	std_srvs::srv::Empty::Response::SharedPtr)
{
	RCLCPP_INFO(nh_->get_logger(), "Clearing turtlesim.");
	clear();
	return true;
}

bool CheckersBoardFrame::resetCallback(
	const std_srvs::srv::Empty::Request::SharedPtr,
	std_srvs::srv::Empty::Response::SharedPtr)
{
	RCLCPP_INFO(nh_->get_logger(), "Resetting checkers.");
	clearPieces();
	spawnPieces();
	clear();
	return true;
}