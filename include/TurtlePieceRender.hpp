#pragma once

#include <QPointF>
#include <QImage>
#include <QPainter>
#include <QVector>

#include "TurtlePiece.hpp"

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
    QImage m_highlightRotatedImage;
    QImage m_selectRotatedImage;
    QImage m_deadRotatedImage;
};

typedef std::shared_ptr<TurtlePieceRender> TurtlePieceRenderPtr;