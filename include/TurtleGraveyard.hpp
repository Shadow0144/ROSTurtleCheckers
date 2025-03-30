#pragma once

#include "CheckersConsts.hpp"
#include "TurtlePieceRender.hpp"

#include <QPainter>
#include <QPen>
#include <QPointF>

#include <memory>
#include <string>
#include <vector>

class TurtleGraveyard
{
public:
    TurtleGraveyard(TurtlePieceColor owningPlayerColor, TurtlePieceColor viewingPlayerColor);

    void addTurtlePiece(TurtlePieceRenderPtr &turtlePieceRender);
    void clear();

    void paint(QPainter &painter) const;

private:
    std::vector<TurtlePieceRenderPtr> m_slainTurtles;

    float m_left;
    QPointF m_nextPosition;
    QPointF m_initialRowPosition;
    QPointF m_positionRowIncrement;
    QPointF m_positionColIncrement;
    size_t m_turtlesInColumn;
};

typedef std::shared_ptr<TurtleGraveyard> TurtleGraveyardPtr;