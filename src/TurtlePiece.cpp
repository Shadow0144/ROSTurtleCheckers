#include "TurtlePiece.hpp"

#include <QColor>
#include <QRgb>

#include <math.h>

#include "rclcpp/rclcpp.hpp"

TurtlePiece::TurtlePiece(
    const std::string &name,
    TurtlePieceColor color,
    const QImage &turtle_image,
    const QImage &king_image,
    const QImage &highlight_image,
    const QImage &select_image,
    const QPointF &position,
    float angle)
    : name_(name),
      color_(color),
      turtle_image_(turtle_image),
      king_image_(king_image),
      highlight_image_(highlight_image),
      select_image_(select_image),
      position_(position),
      angle_(angle)
{
  QTransform transform;
  transform.rotate(-angle * 180.0 / M_PI);
  turtle_rotated_image_ = turtle_image_.transformed(transform);
  king_rotated_image_ = king_image_.transformed(transform);
  highlight_rotated_image_ = highlight_image_.transformed(transform);
  select_rotated_image_ = select_image_.transformed(transform);

  position_.rx() -= 0.5 * turtle_rotated_image_.width();
  position_.ry() -= 0.5 * turtle_rotated_image_.height();

  kinged = false;
  highlighted = false;
  selected = false;
}

std::string &TurtlePiece::getName()
{
  return name_;
}

TurtlePieceColor TurtlePiece::getColor()
{
  return color_;
}

bool TurtlePiece::getKinged()
{
  return kinged;
}

void TurtlePiece::toggleKingship(bool king)
{
  kinged = king;
}

bool TurtlePiece::getHighlighted()
{
  return highlighted;
}

void TurtlePiece::toggleHighlight()
{
  highlighted = !highlighted;
}

void TurtlePiece::toggleHighlight(bool highlight)
{
  highlighted = highlight;
}

bool TurtlePiece::getSelected()
{
  return selected;
}

void TurtlePiece::toggleSelect()
{
  selected = !selected;
}

void TurtlePiece::toggleSelect(bool select)
{
  selected = select;
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
  if (selected)
  {
    painter.drawImage(position_, select_rotated_image_);
  }
}