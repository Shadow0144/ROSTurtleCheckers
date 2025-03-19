#include "turtle_piece.hpp"

#include <QColor>
#include <QRgb>

#include <math.h>

#include "rclcpp/rclcpp.hpp"

TurtlePiece::TurtlePiece(
    const std::string &name,
    const QImage &turtle_image,
    const QPointF &position,
    float angle
    )
    : name_(name),
      turtle_image_(turtle_image),
      position_(position),
      angle_(angle)
{
  QTransform transform;
  transform.rotate(-angle * 180.0 / M_PI);
  turtle_rotated_image_ = turtle_image_.transformed(transform);

  position_.rx() -= 0.5 * turtle_rotated_image_.width();
  position_.ry() -= 0.5 * turtle_rotated_image_.height();
}

void TurtlePiece::move(
  const QPointF &new_position)
{
}

void TurtlePiece::paint(QPainter &painter)
{
  painter.drawImage(position_, turtle_rotated_image_);
}