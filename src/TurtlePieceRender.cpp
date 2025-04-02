#include "TurtlePieceRender.hpp"

#include "ImageLibrary.hpp"

TurtlePieceRender::TurtlePieceRender(
    const std::string &name,
    TurtlePieceColor color,
    const QPointF &centerPosition,
    int angleDegrees)
    : TurtlePiece(name, color),
      m_centerPosition(centerPosition),
      m_angleDegrees(angleDegrees)
{
    updateImages();
    m_centerPosition.rx() -= 0.5 * m_turtleRotatedImage.width();
    m_centerPosition.ry() -= 0.5 * m_turtleRotatedImage.height();
}

void TurtlePieceRender::updateImages()
{
    QTransform transform;
    transform.rotate(m_angleDegrees);
    m_turtleRotatedImage = ImageLibrary::getTurtleImage(m_color).transformed(transform);
    m_kingRotatedImage = ImageLibrary::getKingImage(m_color).transformed(transform);
    m_movableRotatedImage = ImageLibrary::getMovableImage(m_color).transformed(transform);
    m_highlightRotatedImage = ImageLibrary::getHighlightImage(m_color).transformed(transform);
    m_selectRotatedImage = ImageLibrary::getSelectImage(m_color).transformed(transform);
    m_deadRotatedImage = ImageLibrary::getDeadImage(m_color).transformed(transform);
}

void TurtlePieceRender::setCenterPosition(const QPointF &centerPosition)
{
    m_centerPosition = centerPosition;
    m_centerPosition.rx() -= 0.5 * m_turtleRotatedImage.width();
    m_centerPosition.ry() -= 0.5 * m_turtleRotatedImage.height();
}

void TurtlePieceRender::paint(QPainter &painter)
{
    painter.drawImage(m_centerPosition, m_turtleRotatedImage);
    if (m_isKinged)
    {
        painter.drawImage(m_centerPosition, m_kingRotatedImage);
    }
    if (m_isMovable)
    {
        painter.drawImage(m_centerPosition, m_movableRotatedImage);
    }
    if (m_isHighlighted)
    {
        painter.drawImage(m_centerPosition, m_highlightRotatedImage);
    }
    if (m_isSelected)
    {
        painter.drawImage(m_centerPosition, m_selectRotatedImage);
    }
    if (m_isDead)
    {
        painter.drawImage(m_centerPosition, m_deadRotatedImage);
    }
}