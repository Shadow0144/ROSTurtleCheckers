#pragma once

// This prevents a MOC error with versions of boost >= 1.48
#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/twist.hpp>
// #include <turtlesim_msgs/action/rotate_absolute.hpp>
// #include <turtlesim_msgs/msg/color.hpp>
// #include <turtlesim_msgs/msg/pose.hpp>
// #include <turtlesim_msgs/srv/set_pen.hpp>
// #include <turtlesim_msgs/srv/teleport_absolute.hpp>
// #include <turtlesim_msgs/srv/teleport_relative.hpp>
#endif

#include <QImage>
#include <QPainter>
#include <QPen>
#include <QPointF>

#include <memory>
#include <string>
#include <vector>

class Turtle
{
public:
    Turtle(
        rclcpp::Node::SharedPtr &nh, const std::string &real_name, const QImage &turtle_image,
        const QPointF &post);

    bool update(
        double dt, QPainter &path_painter, const QImage &path_image, qreal canvas_width,
        qreal canvas_height);
    void paint(QPainter &painter);

private:
    // bool teleportAbsoluteCallback(
    // const turtlesim_msgs::srv::TeleportAbsolute::Request::SharedPtr,
    // turtlesim_msgs::srv::TeleportAbsolute::Response::SharedPtr);
    // void rotateAbsoluteAcceptCallback(const std::shared_ptr<RotateAbsoluteGoalHandle>);

    rclcpp::Node::SharedPtr nh_;

    QImage turtle_image_;

    QPointF pos_;

    /*
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
    rclcpp::Publisher<turtlesim_msgs::msg::Pose>::SharedPtr pose_pub_;
    rclcpp::Publisher<turtlesim_msgs::msg::Color>::SharedPtr color_pub_;
    rclcpp::Service<turtlesim_msgs::srv::SetPen>::SharedPtr set_pen_srv_;
    rclcpp::Service<turtlesim_msgs::srv::TeleportRelative>::SharedPtr teleport_relative_srv_;
    rclcpp::Service<turtlesim_msgs::srv::TeleportAbsolute>::SharedPtr teleport_absolute_srv_;
    rclcpp_action::Server<turtlesim_msgs::action::RotateAbsolute>
    ::SharedPtr rotate_absolute_action_server_;

    std::shared_ptr<RotateAbsoluteGoalHandle> rotate_absolute_goal_handle_;
    std::shared_ptr<turtlesim_msgs::action::RotateAbsolute::Feedback> rotate_absolute_feedback_;
    std::shared_ptr<turtlesim_msgs::action::RotateAbsolute::Result> rotate_absolute_result_;
    qreal rotate_absolute_start_orient_;

    rclcpp::Time last_command_time_;
    */

    float meter_;

    struct TeleportRequest
    {
        TeleportRequest(float x, float y, qreal _linear, bool _relative)
            : pos(x, y),
              linear(_linear),
              relative(_relative)
        {
        }

        QPointF pos;
        qreal linear;
        bool relative;
    };
    typedef std::vector<TeleportRequest> V_TeleportRequest;
    V_TeleportRequest teleport_requests_;
};

typedef std::shared_ptr<Turtle> TurtlePtr;