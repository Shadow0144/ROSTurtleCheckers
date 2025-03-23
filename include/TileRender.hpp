#pragma once

#include "TurtlePiece.hpp"

#include <QPainter>
#include <QPen>
#include <QPointF>

#include <memory>

class TileRender
{
public:
    TileRender(const QPointF &center_position, 
        size_t tile_width,
        size_t tile_height);

    QPointF getCenterPosition();

    bool getTileHighlighted();

    bool containsPoint(QPoint point);

    void setTurtlePiece(const TurtlePiecePtr &turtle);
    TurtlePiecePtr &getTurtlePiece();

    bool containsPiece(TurtlePiece::TurtleColor color);

    bool togglePieceHighlight();
    void togglePieceHighlight(bool highlight);

    void toggleTileHighlight();
    void toggleTileHighlight(bool highlight);

    void paint(QPainter &painter);

private:
    QPointF center_position_;

    float left;
    float top;
    float right;
    float bottom;

    float tile_width_;
    float tile_height_;

    bool highlighted;

    TurtlePiecePtr contained_turtle;
};

typedef std::shared_ptr<TileRender> TileRenderPtr;