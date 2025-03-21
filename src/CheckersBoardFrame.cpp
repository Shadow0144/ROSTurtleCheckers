#include "CheckersBoardFrame.hpp"

#include <QPointF>

#include <cstdlib>
#include <ctime>
#include <functional>
#include <string>
#include <math.h>

#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rclcpp/rclcpp.hpp"

// Constants
constexpr size_t NUM_PIECES_PER_PLAYER = 12u;

constexpr size_t NUM_COLS_ROWS = 8u;

constexpr int RED_SQUARES_BG_RGB[3] = {255u, 0u, 0u};

constexpr int TILE_WIDTH = 55;
constexpr int TILE_HEIGHT = 55;
constexpr int BOARD_WIDTH = 8 * TILE_WIDTH;
constexpr int BOARD_HEIGHT = 8 * TILE_HEIGHT;

constexpr float DEFAULT_BOARD_SCALE = 1.0f;

CheckersBoardFrame::CheckersBoardFrame(rclcpp::Node::SharedPtr &node_handle, QWidget *parent, Qt::WindowFlags f)
	: QFrame(parent, f),
	  path_image_(BOARD_WIDTH, BOARD_HEIGHT, QImage::Format_ARGB32),
	  path_painter_(&path_image_)
{
	setFixedSize(BOARD_WIDTH, BOARD_HEIGHT);
	setWindowTitle("TurtleCheckers");

	srand(time(NULL));

	update_timer_ = new QTimer(this);
	update_timer_->setInterval(16);
	update_timer_->start();

	connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));

	nh_ = node_handle;

	rcl_interfaces::msg::FloatingPointRange range;
	range.from_value = 0.01f;
	range.to_value = 255.0f;
	rcl_interfaces::msg::ParameterDescriptor board_scale_descriptor;
	board_scale_descriptor.description = "Scale of the checkers board render";
	board_scale_descriptor.floating_point_range.push_back(range);
	nh_->declare_parameter(
		"board_scale", rclcpp::ParameterValue(DEFAULT_BOARD_SCALE), board_scale_descriptor);

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
		QImage b_img;
		b_img.load(images_path + turtles_images[i] + "_black.png");
		black_turtle_images_.append(b_img);
		QImage r_img;
		r_img.load(images_path + turtles_images[i] + "_red.png");
		red_turtle_images_.append(r_img);
		QImage h_img;
		h_img.load(images_path + turtles_images[i] + "_highlight.png");
		highlight_turtle_images_.append(h_img);
		QImage k_img;
		k_img.load(images_path + turtles_images[i] + "_king.png");
		king_turtle_images_.append(k_img);
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

	rclcpp::QoS qos(rclcpp::KeepLast(100), rmw_qos_profile_sensor_data);
	parameter_event_sub_ = nh_->create_subscription<rcl_interfaces::msg::ParameterEvent>(
		"/parameter_events", qos,
		std::bind(&CheckersBoardFrame::parameterEventCallback, this, std::placeholders::_1));

	RCLCPP_INFO(
		nh_->get_logger(), "Starting turtle checkers board with node name %s", nh_->get_fully_qualified_name());

	spawnTiles();
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

void CheckersBoardFrame::spawnTiles()
{
	const float tile_half_width = TILE_WIDTH / 2.0f;
	const float tile_half_height = TILE_HEIGHT / 2.0f;
	float tile_centers_x[NUM_PLAYABLE_TILES];
	float tile_centers_y[NUM_PLAYABLE_TILES];
	for (size_t i = 0u; i < NUM_COLS_ROWS; i++)
	{
		if (i % 2u == 0u)
		{
			tile_centers_x[(i * 4) + 0] = (1 * TILE_WIDTH) + tile_half_width;
			tile_centers_x[(i * 4) + 1] = (3 * TILE_WIDTH) + tile_half_width;
			tile_centers_x[(i * 4) + 2] = (5 * TILE_WIDTH) + tile_half_width;
			tile_centers_x[(i * 4) + 3] = (7 * TILE_WIDTH) + tile_half_width;
		}
		else
		{
			tile_centers_x[(i * 4) + 0] = (0 * TILE_WIDTH) + tile_half_width;
			tile_centers_x[(i * 4) + 1] = (2 * TILE_WIDTH) + tile_half_width;
			tile_centers_x[(i * 4) + 2] = (4 * TILE_WIDTH) + tile_half_width;
			tile_centers_x[(i * 4) + 3] = (6 * TILE_WIDTH) + tile_half_width;
		}

		tile_centers_y[(i * 4) + 0] = (i * TILE_HEIGHT) + tile_half_height;
		tile_centers_y[(i * 4) + 1] = (i * TILE_HEIGHT) + tile_half_height;
		tile_centers_y[(i * 4) + 2] = (i * TILE_HEIGHT) + tile_half_height;
		tile_centers_y[(i * 4) + 3] = (i * TILE_HEIGHT) + tile_half_height;
	}
	for (size_t i = 0u; i < NUM_PLAYABLE_TILES; i++)
	{
		tile_renders[i] = std::make_shared<TileRender>(QPointF(tile_centers_x[i], tile_centers_y[i]), TILE_WIDTH, TILE_HEIGHT);
	}
}

void CheckersBoardFrame::clearPieces()
{
}

void CheckersBoardFrame::spawnPieces()
{
	// Spawn all the checkers pieces
	// Black
	for (size_t i = 0u; i < NUM_PIECES_PER_PLAYER; i++)
	{
		auto center = tile_renders[i]->getCenterPosition();
		spawnTurtle("Black" + std::to_string(i + 1),
					false,
					center.x(),
					center.y(),
					1.0f * M_PI,
					black_image_index);
	}
	// Red
	for (size_t i = 0u; i < NUM_PIECES_PER_PLAYER; i++)
	{
		auto center = tile_renders[i + 20]->getCenterPosition();
		spawnTurtle("Red" + std::to_string(i + 1),
					true,
					center.x(),
					center.y(),
					0.0f * M_PI,
					red_image_index);
	}
}

std::string CheckersBoardFrame::spawnTurtle(const std::string &name,
											bool black,
											float x,
											float y,
											float angle,
											size_t image_index)
{
	if (black)
	{
		black_turtles_[name] = std::make_shared<TurtlePiece>(
			name,
			black_turtle_images_[static_cast<int>(image_index)],
			highlight_turtle_images_[static_cast<int>(image_index)],
			king_turtle_images_[static_cast<int>(image_index)],
			QPointF(x, y),
			angle);
		RCLCPP_INFO(
			nh_->get_logger(), "Spawning black turtle [%s] at x=[%f], y=[%f], theta=[%f]",
			name.c_str(), x, y, angle);
	}
	else // red
	{
		red_turtles_[name] = std::make_shared<TurtlePiece>(
			name,
			red_turtle_images_[static_cast<int>(image_index)],
			highlight_turtle_images_[static_cast<int>(image_index)],
			king_turtle_images_[static_cast<int>(image_index)],
			QPointF(x, y),
			angle);
		RCLCPP_INFO(
			nh_->get_logger(), "Spawning red turtle [%s] at x=[%f], y=[%f], theta=[%f]",
			name.c_str(), x, y, angle);
	}
	update();

	return name;
}

void CheckersBoardFrame::clear()
{
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

	// Fill the background in red
	QRgb background_color = qRgb(RED_SQUARES_BG_RGB[0], RED_SQUARES_BG_RGB[1], RED_SQUARES_BG_RGB[2]);
	painter.fillRect(0, 0, width(), height(), background_color);

	// Draw the black tiles over the red background
	for (size_t i = 0u; i < NUM_PLAYABLE_TILES; i++)
	{
		tile_renders[i]->paint(painter);
	}

	// Draw the black turtles
	TurtlePiecesMap::iterator bit = black_turtles_.begin();
	TurtlePiecesMap::iterator bend = black_turtles_.end();
	for (; bit != bend; ++bit)
	{
		bit->second->paint(painter);
	}

	// Draw the red turtles
	TurtlePiecesMap::iterator rit = red_turtles_.begin();
	TurtlePiecesMap::iterator rend = red_turtles_.end();
	for (; rit != rend; ++rit)
	{
		rit->second->paint(painter);
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