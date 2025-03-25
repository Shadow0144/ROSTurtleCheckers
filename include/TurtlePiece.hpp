#pragma once

#include <QImage>
#include <QPainter>
#include <QPen>
#include <QPointF>

#include <memory>
#include <string>
#include <vector>

#include "CheckersConsts.hpp"

class TurtlePiece
{
public:
    TurtlePiece(
        const std::string &name,
        TurtlePieceColor color,
        const QImage &turtleImage,
        const QImage &kingImage,
        const QImage &highlightImage,
        const QImage &selectImage,
        const QPointF &position,
        int angleDegrees);

    std::string &getName();

    TurtlePieceColor getColor();

    bool getIsKinged();
    void toggleIsKinged();
    void toggleIsKinged(bool isKinged);

    bool getIsHighlighted();
    void toggleIsHighlighted();
    void toggleIsHighlighted(bool isHighlighted);

    bool getIsSelected();
    void toggleIsSelected();
    void toggleIsSelected(bool isSelected);

    void move(const QPointF &newPosition);

    void paint(QPainter &painter);

private:
    std::string m_name;

    TurtlePieceColor m_color;

    QImage m_turtleImage;
    QImage m_turtleRotatedImage;
    QImage m_kingImage;
    QImage m_kingRotatedImage;
    QImage m_highlightImage;
    QImage m_highlightRotatedImage;
    QImage m_selectImage;
    QImage m_selectRotatedImage;

    QPointF m_position;
    int m_angleDegrees; // In degrees

    bool m_isKinged;
    bool m_isHighlighted;
    bool m_isSelected;
};

typedef std::shared_ptr<TurtlePiece> TurtlePiecePtr;