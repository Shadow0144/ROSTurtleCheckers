#include "TurtlePiece.hpp"

#include <QColor>
#include <QRgb>

#include <math.h>

#include "rclcpp/rclcpp.hpp"

TurtlePiece::TurtlePiece(
    const std::string &name,
    TurtlePiece::TurtleColor color,
    const QImage &turtle_image,
    const QImage &highlight_image,
    const QImage &king_image,
    const QPointF &position,
    float angle)
    : name_(name),
      color_(color),
      turtle_image_(turtle_image),
      highlight_image_(highlight_image),
      king_image_(king_image),
      position_(position),
      angle_(angle)
{
  QTransform transform;
  transform.rotate(-angle * 180.0 / M_PI);
  turtle_rotated_image_ = turtle_image_.transformed(transform);
  highlight_rotated_image_ = highlight_image_.transformed(transform);
  king_rotated_image_ = king_image_.transformed(transform);

  position_.rx() -= 0.5 * turtle_rotated_image_.width();
  position_.ry() -= 0.5 * turtle_rotated_image_.height();

  highlighted = false;
  kinged = false;
}

TurtlePiece::TurtleColor TurtlePiece::getColor()
{
  return color_;
}

bool TurtlePiece::getHighlighted()
{
  return highlighted;
}

bool TurtlePiece::getKinged()
{
  return kinged;
}

void TurtlePiece::toggleHighlight()
{
  highlighted = !highlighted;
}

void TurtlePiece::toggleHighlight(bool highlight)
{
  highlighted = highlight;
}

void TurtlePiece::toggleKingship(bool king)
{
  kinged = king;
}

void TurtlePiece::move(
    const QPointF &new_position)
{
  position_ = new_position;
  position_.rx() -= 0.5 * turtle_rotated_image_.width();
  position_.ry() -= 0.5 * turtle_rotated_image_.height();
}

void TurtlePiece::paint(QPainter &painter)
{
  painter.drawImage(position_, turtle_rotated_image_);
  if (kinged)
  {
    painter.drawImage(position_, king_rotated_image_);
  }
  if (highlighted)
  {
    painter.drawImage(position_, highlight_rotated_image_);
  }
}