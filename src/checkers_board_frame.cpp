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

#define DEFAULT_BG_R 0x45
#define DEFAULT_BG_G 0x56
#define DEFAULT_BG_B 0xff

CheckersBoardFrame::CheckersBoardFrame(rclcpp::Node::SharedPtr &node_handle, QWidget *parent, Qt::WindowFlags f)
	: QFrame(parent, f), path_image_(500, 500, QImage::Format_ARGB32), path_painter_(&path_image_), frame_count_(0), id_counter_(0)
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
		"background_r", rclcpp::ParameterValue(DEFAULT_BG_R), background_r_descriptor);
	nh_->declare_parameter(
		"background_g", rclcpp::ParameterValue(DEFAULT_BG_G), background_g_descriptor);
	nh_->declare_parameter(
		"background_b", rclcpp::ParameterValue(DEFAULT_BG_B), background_b_descriptor);

	rcl_interfaces::msg::ParameterDescriptor holonomic_descriptor;
	holonomic_descriptor.description = "If true, then turtles will be holonomic";
	nh_->declare_parameter("holonomic", rclcpp::ParameterValue(false), holonomic_descriptor);

	QVector<QString> turtles;
	/*turtles.append("ardent.png");
	turtles.append("bouncy.png");
	turtles.append("crystal.png");
	turtles.append("dashing.png");
	turtles.append("eloquent.png");
	turtles.append("foxy.png");
	turtles.append("galactic.png");
	turtles.append("humble.png");
	turtles.append("iron.png");
	turtles.append("jazzy.png");*/
	turtles.append("rolling");

	QString images_path =
		(ament_index_cpp::get_package_share_directory("turtle_checkers") + "/img/").c_str();
	for (int i = 0; i < turtles.size(); ++i)
	{
		QImage r_img;
		r_img.load(images_path + turtles[i] + "_red.png");
		red_turtle_images_.append(r_img);
		QImage b_img;
		b_img.load(images_path + turtles[i] + "_black.png");
		black_turtle_images_.append(b_img);
	}

	meter_ = red_turtle_images_[0].height(); // Assume we have at least one red image

	clear();

	clear_srv_ =
		nh_->create_service<std_srvs::srv::Empty>(
			"clear",
			std::bind(&CheckersBoardFrame::clearCallback, this, std::placeholders::_1, std::placeholders::_2));
	reset_srv_ =
		nh_->create_service<std_srvs::srv::Empty>(
			"reset",
			std::bind(&CheckersBoardFrame::resetCallback, this, std::placeholders::_1, std::placeholders::_2));
	/*spawn_srv_ =
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

	width_in_meters_ = (width() - 1) / meter_;
	height_in_meters_ = (height() - 1) / meter_;
	spawnRedTurtle("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0);

	// Spawn all available turtle types
	if (false)
	{
		for (int index = 0; index < turtles.size(); ++index)
		{
			QString name = turtles[index];
			name = name.split(".").first();
			name.replace(QString("-"), QString(""));
			spawnRedTurtle(
				name.toStdString(), 1.0f + 1.5f * (index % 7), 1.0f + 1.5f * (index / 7),
				static_cast<float>(M_PI) / 2.0f, index);
		}
	}
}

CheckersBoardFrame::~CheckersBoardFrame()
{
	delete update_timer_;
}
/*
bool CheckersBoardFrame::spawnCallback(
  const turtlesim_msgs::srv::Spawn::Request::SharedPtr req,
  turtlesim_msgs::srv::Spawn::Response::SharedPtr res)
{
  std::string name = spawnTurtle(req->name, req->x, req->y, req->theta);
  if (name.empty()) {
	RCLCPP_ERROR(nh_->get_logger(), "A turtle named [%s] already exists", req->name.c_str());
	return false;
  }

  res->name = name;

  return true;
}

bool CheckersBoardFrame::killCallback(
  const turtlesim_msgs::srv::Kill::Request::SharedPtr req,
  turtlesim_msgs::srv::Kill::Response::SharedPtr)
{
  M_Turtle::iterator it = turtles_.find(req->name);
  if (it == turtles_.end()) {
	RCLCPP_ERROR(
	  nh_->get_logger(), "Tried to kill turtle [%s], which does not exist", req->name.c_str());
	return false;
  }

  turtles_.erase(it);
  update();

  return true;
}
*/
void CheckersBoardFrame::parameterEventCallback(
	const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr event)
{
	// only consider events from this node
	if (event->node == nh_->get_fully_qualified_name())
	{
		// since parameter events for this event aren't expected frequently just always call update()
		update();
	}
}

std::string CheckersBoardFrame::spawnRedTurtle(const std::string &name, float x, float y, float angle)
{
	return spawnRedTurtle(name, x, y, angle, rand() % red_turtle_images_.size());
}

std::string CheckersBoardFrame::spawnRedTurtle(
	const std::string &name, float x, float y, float angle,
	size_t index)
{
	/*std::string real_name = name;
	if (real_name.empty()) {
	  do{
		std::stringstream ss;
		ss << "turtle" << ++id_counter_;
		real_name = ss.str();
	  } while (hasTurtle(real_name));
	} else {
	  if (hasTurtle(real_name)) {
		return "";
	  }
	}

	TurtlePtr t = std::make_shared<Turtle>(
	  nh_, real_name, turtle_images_[static_cast<int>(index)], QPointF(
		x,
		height_in_meters_ - y), angle);
	turtles_[real_name] = t;
	update();

	RCLCPP_INFO(
	  nh_->get_logger(), "Spawning turtle [%s] at x=[%f], y=[%f], theta=[%f]",
	  real_name.c_str(), x, y, angle);

	return real_name;*/
	return "";
}

std::string CheckersBoardFrame::spawnBlackTurtle(const std::string &name, float x, float y, float angle)
{
	return spawnBlackTurtle(name, x, y, angle, rand() % black_turtle_images_.size());
}

std::string CheckersBoardFrame::spawnBlackTurtle(
	const std::string &name, float x, float y, float angle,
	size_t index)
{
	/*std::string real_name = name;
	if (real_name.empty()) {
	  do{
		std::stringstream ss;
		ss << "turtle" << ++id_counter_;
		real_name = ss.str();
	  } while (hasTurtle(real_name));
	} else {
	  if (hasTurtle(real_name)) {
		return "";
	  }
	}

	TurtlePtr t = std::make_shared<Turtle>(
	  nh_, real_name, turtle_images_[static_cast<int>(index)], QPointF(
		x,
		height_in_meters_ - y), angle);
	turtles_[real_name] = t;
	update();

	RCLCPP_INFO(
	  nh_->get_logger(), "Spawning turtle [%s] at x=[%f], y=[%f], theta=[%f]",
	  real_name.c_str(), x, y, angle);

	return real_name;*/
	return "";
}

void CheckersBoardFrame::clear()
{
	// make all pixels fully transparent
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

	int r = DEFAULT_BG_R;
	int g = DEFAULT_BG_G;
	int b = DEFAULT_BG_B;
	nh_->get_parameter("background_r", r);
	nh_->get_parameter("background_g", g);
	nh_->get_parameter("background_b", b);
	QRgb background_color = qRgb(r, g, b);
	painter.fillRect(0, 0, width(), height(), background_color);

	painter.drawImage(QPoint(0, 0), path_image_);

	/*M_Turtle::iterator it = turtles_.begin();
	M_Turtle::iterator end = turtles_.end();
	for (; it != end; ++it) {
	  it->second->paint(painter);
	}*/
}

/*void CheckersBoardFrame::updateTurtles()
{
  if (last_turtle_update_.nanoseconds() == 0) {
	last_turtle_update_ = nh_->now();
	return;
  }

  bool modified = false;
  M_Turtle::iterator it = turtles_.begin();
  M_Turtle::iterator end = turtles_.end();
  for (; it != end; ++it) {
	modified |= it->second->update(
	  0.001 * update_timer_->interval(), path_painter_, path_image_, width_in_meters_,
	  height_in_meters_);
  }
  if (modified) {
	update();
  }

  ++frame_count_;
}*/

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
	// turtles_.clear();
	id_counter_ = 0;
	// spawnTurtle("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0);
	clear();
	return true;
}