#include "TurtlePiece.hpp"

#include "CheckersConsts.hpp"

TurtlePiece::TurtlePiece(
    const std::string &name,
    TurtlePieceColor color)
    : m_name(name),
      m_color(color)
{
  m_isHighlighted = false;
  m_isSelected = false;
  m_isKinged = false;
  m_isDead = false;
}

std::string &TurtlePiece::getName()
{
  return m_name;
}

TurtlePieceColor TurtlePiece::getColor()
{
  return m_color;
}

bool TurtlePiece::getIsHighlighted()
{
  return m_isHighlighted;
}

void TurtlePiece::setIsHighlighted(bool isHighlighted)
{
  m_isHighlighted = isHighlighted;
}

void TurtlePiece::toggleIsHighlighted()
{
  m_isHighlighted = !m_isHighlighted;
}

bool TurtlePiece::getIsSelected()
{
  return m_isSelected;
}
void TurtlePiece::setIsSelected(bool isSelected)
{
  m_isSelected = isSelected;
}

void TurtlePiece::toggleIsSelected()
{
  m_isSelected = !m_isSelected;
}

bool TurtlePiece::getIsKinged()
{
  return m_isKinged;
}

void TurtlePiece::setIsKinged(bool isKinged)
{
  m_isKinged = isKinged;
}

void TurtlePiece::toggleIsKinged()
{
  m_isKinged = !m_isKinged;
}

bool TurtlePiece::getIsDead()
{
  return m_isDead;
}

void TurtlePiece::setIsDead(bool isDead)
{
  m_isDead = isDead;
}

void TurtlePiece::toggleIsDead()
{
  m_isDead = !m_isDead;
}