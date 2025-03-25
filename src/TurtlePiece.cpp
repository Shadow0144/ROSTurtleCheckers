#include "TurtlePiece.hpp"

#include <QColor>
#include <QRgb>

#include "rclcpp/rclcpp.hpp"

#include "CheckersConsts.hpp"

TurtlePiece::TurtlePiece(
    const std::string &name,
    TurtlePieceColor color,
    const QImage &turtleImage,
    const QImage &kingImage,
    const QImage &highlightImage,
    const QImage &selectImage,
    const QPointF &position,
    int angleDegrees)
    : m_name(name),
      m_color(color),
      m_turtleImage(turtleImage),
      m_kingImage(kingImage),
      m_highlightImage(highlightImage),
      m_selectImage(selectImage),
      m_position(position),
      m_angleDegrees(angleDegrees)
{
  QTransform transform;
  transform.rotate(m_angleDegrees);
  m_turtleRotatedImage = m_turtleImage.transformed(transform);
  m_kingRotatedImage = m_kingImage.transformed(transform);
  m_highlightRotatedImage = m_highlightImage.transformed(transform);
  m_selectRotatedImage = m_selectImage.transformed(transform);

  m_position.rx() -= 0.5 * m_turtleRotatedImage.width();
  m_position.ry() -= 0.5 * m_turtleRotatedImage.height();

  m_isKinged = false;
  m_isHighlighted = false;
  m_isSelected = false;
}

std::string &TurtlePiece::getName()
{
  return m_name;
}

TurtlePieceColor TurtlePiece::getColor()
{
  return m_color;
}

bool TurtlePiece::getIsKinged()
{
  return m_isKinged;
}

void TurtlePiece::toggleIsKinged()
{
  m_isKinged = !m_isKinged;
}

void TurtlePiece::toggleIsKinged(bool isKinged)
{
  m_isKinged = isKinged;
}

bool TurtlePiece::getIsHighlighted()
{
  return m_isHighlighted;
}

void TurtlePiece::toggleIsHighlighted()
{
  m_isHighlighted = !m_isHighlighted;
}

void TurtlePiece::toggleIsHighlighted(bool isHighlighted)
{
  m_isHighlighted = isHighlighted;
}

bool TurtlePiece::getIsSelected()
{
  return m_isSelected;
}

void TurtlePiece::toggleIsSelected()
{
  m_isSelected = !m_isSelected;
}

void TurtlePiece::toggleIsSelected(bool isSelected)
{
  m_isSelected = isSelected;
}

void TurtlePiece::move(
    const QPointF &newPosition)
{
  m_position = newPosition;
  m_position.rx() -= 0.5 * m_turtleRotatedImage.width();
  m_position.ry() -= 0.5 * m_turtleRotatedImage.height();
}

void TurtlePiece::paint(QPainter &painter)
{
  painter.drawImage(m_position, m_turtleRotatedImage);
  if (m_isKinged)
  {
    painter.drawImage(m_position, m_kingRotatedImage);
  }
  if (m_isHighlighted)
  {
    painter.drawImage(m_position, m_highlightRotatedImage);
  }
  if (m_isSelected)
  {
    painter.drawImage(m_position, m_selectRotatedImage);
  }
}