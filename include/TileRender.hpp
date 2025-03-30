#pragma once

#include "Tile.hpp"

#include "CheckersConsts.hpp"
#include "TurtlePieceRender.hpp"
#include "TurtleGraveyard.hpp"

#include <QPainter>
#include <QPointF>

#include <memory>

class TileRender : public Tile
{
public:
    TileRender(int row, int col, const QPointF &centerPosition);

    const QPointF &getCenterPosition() const;

    bool containsPoint(const QPointF &point) const;

    void setTurtlePiece(const TurtlePiecePtr &turtle) override;

    void moveTurtlePiece(const std::shared_ptr<TileRender> &destinationTileRender);
    void moveTurtlePiece(const TurtleGraveyardPtr &destinationGraveyard);

    void paint(QPainter &painter) const;

private:
    QPointF m_centerPosition;

    float m_left;
    float m_top;
    float m_right;
    float m_bottom;
};

typedef std::shared_ptr<TileRender> TileRenderPtr;