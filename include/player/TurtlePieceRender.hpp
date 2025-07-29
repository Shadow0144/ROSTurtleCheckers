#pragma once

#include <QPointF>
#include <QImage>
#include <QPainter>
#include <QVector>

#include "shared/CheckersConsts.hpp"
#include "shared/TurtlePiece.hpp"

class TurtlePieceRender : public TurtlePiece
{
public:
    TurtlePieceRender(
        const std::string &name,
        TurtlePieceColor color,
        const QPointF &centerPosition, 
        int angleDegrees);

    void updateImages(); // Call if the image index changes

    void setCenterPosition(const QPointF &centerPosition);
    
    void paint(QPainter &painter);

private:
    QPointF m_centerPosition;
    int m_angleDegrees;
    
    QImage m_turtleRotatedImage;
    QImage m_kingRotatedImage;
    QImage m_movableRotatedImage;
    QImage m_highlightRotatedImage;
    QImage m_selectRotatedImage;
    QImage m_deadRotatedImage;

    friend struct std::hash<std::shared_ptr<TurtlePieceRender>>;
};

typedef std::shared_ptr<TurtlePieceRender> TurtlePieceRenderPtr;

template <>
struct std::hash<TurtlePieceRenderPtr>
{
    size_t operator()(const TurtlePieceRenderPtr &turtlePieceRenderPtr) const noexcept;
};