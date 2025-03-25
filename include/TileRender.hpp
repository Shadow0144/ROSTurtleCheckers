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
    TileRender(const QPointF &centerPosition);

    QPointF getCenterPosition();

    bool containsPoint(QPoint point);

    void setTurtlePiece(const TurtlePiecePtr &turtle);
    TurtlePiecePtr &getTurtlePiece();

    bool containsPiece(TurtlePieceColor color);

    bool getIsPieceHighlighted();
    bool toggleIsPieceHighlighted();
    void toggleIsPieceHighlighted(bool isHighlighted);

    bool getIsPieceSelected();
    bool toggleIsPieceSelected();
    void toggleIsPieceSelected(bool isSelected);

    bool getIsTileReachable();
    void toggleIsTileReachable();
    void toggleIsTileReachable(bool isReachable);

    bool getIsTileHighlighted();
    void toggleIsTileHighlighted();
    void toggleIsTileHighlighted(bool isHighlighted);

    bool getIsTileSelected();
    void toggleIsTileSelected();
    void toggleIsTileSelected(bool isSelected);

    void paint(QPainter &painter);

private:
    QPointF m_centerPosition;

    float m_left;
    float m_top;
    float m_right;
    float m_bottom;

    bool m_isReachable;
    bool m_isHighlighted;
    bool m_isSelected;

    TurtlePiecePtr m_containedTurtle;
};

typedef std::shared_ptr<TileRender> TileRenderPtr;