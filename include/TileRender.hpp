#pragma once

#include "CheckersConsts.hpp"
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

    bool containsPoint(QPoint point);

    void setTurtlePiece(const TurtlePiecePtr &turtle);
    TurtlePiecePtr &getTurtlePiece();

    bool containsPiece(TurtlePieceColor color);

    bool getPieceHighlight();
    bool togglePieceHighlight();
    void togglePieceHighlight(bool highlight);

    bool getPieceSelect();
    bool togglePieceSelect();
    void togglePieceSelect(bool select);

    bool getTileReachable();
    void toggleTileReachable();
    void toggleTileReachable(bool reachable);

    bool getTileHighlight();
    void toggleTileHighlight();
    void toggleTileHighlight(bool highlight);

    bool getTileSelect();
    void toggleTileSelect();
    void toggleTileSelect(bool select);

    void paint(QPainter &painter);

private:
    QPointF center_position_;

    float left;
    float top;
    float right;
    float bottom;

    float tile_width_;
    float tile_height_;

    bool reachabled;
    bool highlighted;
    bool selected;

    TurtlePiecePtr contained_turtle;
};

typedef std::shared_ptr<TileRender> TileRenderPtr;