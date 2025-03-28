#pragma once

#include "CheckersConsts.hpp"
#include "TileRender.hpp"
#include "TurtlePiece.hpp"

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

    void addTurtlePiece(const std::shared_ptr<TileRender> &tile);
    void clear();

    void paint(QPainter &painter);

private:
    std::vector<TurtlePiecePtr> m_slainTurtles;

    float m_left;
    QPointF m_nextPosition;
    QPointF m_positionIncrement;
};

typedef std::shared_ptr<TurtleGraveyard> TurtleGraveyardPtr;