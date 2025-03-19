#include "turtle.hpp"

#include <QColor>
#include <QRgb>

#include <cmath>
#include <functional>
#include <string>
#include <math.h>

// #include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

// #include "turtlesim_msgs/action/rotate_absolute.hpp"
// #include "turtlesim_msgs/msg/pose.hpp"
// #include "turtlesim_msgs/msg/color.hpp"
// #include "turtlesim_msgs/srv/set_pen.hpp"
// #include "turtlesim_msgs/srv/teleport_absolute.hpp"
// #include "turtlesim_msgs/srv/teleport_relative.hpp"
// #include "turtlesim/qos.hpp"

Turtle::Turtle(
    rclcpp::Node::SharedPtr &nh, const std::string &real_name,
    const QImage &turtle_image, const QPointF &pos)
    : nh_(nh),
      turtle_image_(turtle_image),
      pos_(pos)
{
  // const rclcpp::QoS qos = topic_qos();
  /*velocity_sub_ = nh_->create_subscription<geometry_msgs::msg::Twist>(
    real_name + "/cmd_vel", qos, std::bind(
      &Turtle::velocityCallback, this,
      std::placeholders::_1));
  pose_pub_ = nh_->create_publisher<turtlesim_msgs::msg::Pose>(real_name + "/pose", qos);
  color_pub_ = nh_->create_publisher<turtlesim_msgs::msg::Color>(real_name + "/color_sensor", qos);
  set_pen_srv_ =
    nh_->create_service<turtlesim_msgs::srv::SetPen>(
    real_name + "/set_pen",
    std::bind(&Turtle::setPenCallback, this, std::placeholders::_1, std::placeholders::_2));
  teleport_relative_srv_ = nh_->create_service<turtlesim_msgs::srv::TeleportRelative>(
    real_name + "/teleport_relative",
    std::bind(
      &Turtle::teleportRelativeCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  teleport_absolute_srv_ = nh_->create_service<turtlesim_msgs::srv::TeleportAbsolute>(
    real_name + "/teleport_absolute",
    std::bind(
      &Turtle::teleportAbsoluteCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  rotate_absolute_action_server_ =
    rclcpp_action::create_server<turtlesim_msgs::action::RotateAbsolute>(
      nh,
      real_name + "/rotate_absolute",
    [](const rclcpp_action::GoalUUID &,
    std::shared_ptr<const turtlesim_msgs::action::RotateAbsolute::Goal>)
    {
        // Accept all goals
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
    [](const std::shared_ptr<RotateAbsoluteGoalHandle>)
    {
        // Accept all cancel requests
      return rclcpp_action::CancelResponse::ACCEPT;
      },
      std::bind(&Turtle::rotateAbsoluteAcceptCallback, this, std::placeholders::_1));*/

  // last_command_time_ = nh_->now();

  meter_ = turtle_image_.height();
}
/*
bool Turtle::teleportAbsoluteCallback(
  const turtlesim_msgs::srv::TeleportAbsolute::Request::SharedPtr req,
  turtlesim_msgs::srv::TeleportAbsolute::Response::SharedPtr)
{
  teleport_requests_.push_back(TeleportRequest(req->x, req->y, req->theta, 0, false));
  return true;
}

void Turtle::rotateAbsoluteAcceptCallback(
  const std::shared_ptr<RotateAbsoluteGoalHandle> goal_handle)
{
  // Abort any existing goal
  if (rotate_absolute_goal_handle_) {
    RCLCPP_WARN(
      nh_->get_logger(),
      "Rotation goal received before a previous goal finished. Aborting previous goal");
    rotate_absolute_goal_handle_->abort(rotate_absolute_result_);
  }
  rotate_absolute_goal_handle_ = goal_handle;
  rotate_absolute_feedback_.reset(new turtlesim_msgs::action::RotateAbsolute::Feedback);
  rotate_absolute_result_.reset(new turtlesim_msgs::action::RotateAbsolute::Result);
  rotate_absolute_start_orient_ = orient_;
}
*/

bool Turtle::update(
    double dt, QPainter &path_painter, const QImage &path_image,
    qreal canvas_width, qreal canvas_height)
{
  /*bool modified = false;
  qreal old_orient = orient_;

  // first process any teleportation requests, in order
  V_TeleportRequest::iterator it = teleport_requests_.begin();
  V_TeleportRequest::iterator end = teleport_requests_.end();
  for (; it != end; ++it) {
    const TeleportRequest & req = *it;

    QPointF old_pos = pos_;
    if (req.relative) {
      orient_ += req.theta;
      pos_.rx() += std::cos(orient_) * req.linear;
      pos_.ry() += -std::sin(orient_) * req.linear;
    } else {
      pos_.setX(req.pos.x());
      pos_.setY(std::max(0.0, static_cast<double>(canvas_height - req.pos.y())));
      orient_ = req.theta;
    }

    if (pen_on_) {
      path_painter.setPen(pen_);
      path_painter.drawLine(pos_ * meter_, old_pos * meter_);
    }
    modified = true;
  }

  teleport_requests_.clear();

  // Process any action requests
  if (rotate_absolute_goal_handle_) {
    // Check if there was a cancel request
    if (rotate_absolute_goal_handle_->is_canceling()) {
      RCLCPP_INFO(nh_->get_logger(), "Rotation goal canceled");
      rotate_absolute_goal_handle_->canceled(rotate_absolute_result_);
      rotate_absolute_goal_handle_ = nullptr;
      lin_vel_x_ = 0.0;
      lin_vel_y_ = 0.0;
      ang_vel_ = 0.0;
    } else {
      double theta = normalizeAngle(rotate_absolute_goal_handle_->get_goal()->theta);
      double remaining = normalizeAngle(theta - static_cast<float>(orient_));

      // Update result
      rotate_absolute_result_->delta =
        normalizeAngle(static_cast<float>(rotate_absolute_start_orient_ - orient_));

      // Update feedback
      rotate_absolute_feedback_->remaining = remaining;
      rotate_absolute_goal_handle_->publish_feedback(rotate_absolute_feedback_);

      // Check stopping condition
      if (fabs(normalizeAngle(static_cast<float>(orient_) - theta)) < 0.02) {
        RCLCPP_INFO(nh_->get_logger(), "Rotation goal completed successfully");
        rotate_absolute_goal_handle_->succeed(rotate_absolute_result_);
        rotate_absolute_goal_handle_ = nullptr;
        lin_vel_x_ = 0.0;
        lin_vel_y_ = 0.0;
        ang_vel_ = 0.0;
      } else {
        lin_vel_x_ = 0.0;
        lin_vel_y_ = 0.0;
        ang_vel_ = remaining < 0.0 ? -1.0 : 1.0;
        last_command_time_ = nh_->now();
      }
    }
  }

  if (nh_->now() - last_command_time_ > rclcpp::Duration(1.0, 0)) {
    lin_vel_x_ = 0.0;
    lin_vel_y_ = 0.0;
    ang_vel_ = 0.0;
  }

  QPointF old_pos = pos_;

  orient_ = orient_ + ang_vel_ * dt;
  // Keep orient_ between -pi and +pi
  orient_ = normalizeAngle(orient_);
  pos_.rx() += std::cos(orient_) * lin_vel_x_ * dt -
    std::sin(orient_) * lin_vel_y_ * dt;
  pos_.ry() -= std::cos(orient_) * lin_vel_y_ * dt +
    std::sin(orient_) * lin_vel_x_ * dt;

  // Clamp to screen size
  if (pos_.x() < 0 || pos_.x() > canvas_width ||
    pos_.y() < 0 || pos_.y() > canvas_height)
  {
    RCLCPP_WARN(
      nh_->get_logger(), "Oh no! I hit the wall! (Clamping from [x=%f, y=%f])",
      pos_.x(), pos_.y());
  }

  pos_.setX(
    std::min(
      std::max(
        static_cast<double>(pos_.x()), 0.0), static_cast<double>(canvas_width)));
  pos_.setY(
    std::min(
      std::max(static_cast<double>(pos_.y()), 0.0),
      static_cast<double>(canvas_height)));

  // Publish pose of the turtle
  auto p = std::make_unique<turtlesim_msgs::msg::Pose>();
  p->x = pos_.x();
  p->y = canvas_height - pos_.y();
  p->theta = orient_;
  p->linear_velocity = std::sqrt(lin_vel_x_ * lin_vel_x_ + lin_vel_y_ * lin_vel_y_);
  p->angular_velocity = ang_vel_;
  pose_pub_->publish(std::move(p));

  // Figure out (and publish) the color underneath the turtle
  {
    auto color = std::make_unique<turtlesim_msgs::msg::Color>();
    QRgb pixel = path_image.pixel((pos_ * meter_).toPoint());
    color->r = qRed(pixel);
    color->g = qGreen(pixel);
    color->b = qBlue(pixel);
    color_pub_->publish(std::move(color));
  }

  RCLCPP_DEBUG(
    nh_->get_logger(), "[%s]: pos_x: %f pos_y: %f theta: %f",
    nh_->get_namespace(), pos_.x(), pos_.y(), orient_);

  if (orient_ != old_orient) {
    rotateImage();
    modified = true;
  }
  if (pos_ != old_pos) {
    if (pen_on_) {
      path_painter.setPen(pen_);
      path_painter.drawLine(pos_ * meter_, old_pos * meter_);
    }
    modified = true;
  }
    */
  // return modified;
  return true;
}

void Turtle::paint(QPainter &painter)
{
  QPointF p = pos_ * meter_;
  p.rx() -= 0.5 * turtle_image_.width();
  p.ry() -= 0.5 * turtle_image_.height();
  painter.drawImage(p, turtle_image_);
}