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

    QPointF getCenterPosition() const;

    bool containsPoint(QPoint point) const;

    void setTurtlePiece(const TurtlePiecePtr &turtle);
    const TurtlePiecePtr &getTurtlePiece() const;
    TurtlePieceColor getTurtlePieceColor() const;
    void moveTurtlePiece(const std::shared_ptr<TileRender> &destinationTile);
    void clearTurtlePiece();

    bool containsPiece(TurtlePieceColor color) const;

    void kingTurtlePiece();

    bool getIsPieceHighlighted() const;
    bool toggleIsPieceHighlighted();
    void toggleIsPieceHighlighted(bool isHighlighted);

    bool getIsPieceSelected() const;
    bool toggleIsPieceSelected();
    void toggleIsPieceSelected(bool isSelected);

    bool getIsTileReachable() const;
    void toggleIsTileReachable();
    void toggleIsTileReachable(bool isReachable);

    bool getIsTileHighlighted() const;
    void toggleIsTileHighlighted();
    void toggleIsTileHighlighted(bool isHighlighted);

    bool getIsTileSelected() const;
    void toggleIsTileSelected();
    void toggleIsTileSelected(bool isSelected);

    bool getIsTileLastSelected() const;
    void toggleIsTileLastSelected();
    void toggleIsTileLastSelected(bool isLastSelected);

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
    bool m_isLastSelected;

    TurtlePiecePtr m_containedTurtle;
};

typedef std::shared_ptr<TileRender> TileRenderPtr;