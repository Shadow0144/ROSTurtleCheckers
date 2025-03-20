#pragma once

#include <QImage>
#include <QPainter>
#include <QPen>
#include <QPointF>

#include <memory>
#include <string>
#include <vector>

class TurtlePiece
{
public:
    TurtlePiece(
        const std::string &name, 
        const QImage &turtle_image, 
        const QPointF &position,
        float angle);

    void move(const QPointF &new_position);
    void paint(QPainter &painter);

private:
    std::string name_;

    QImage turtle_image_;
    QImage turtle_rotated_image_;

    QPointF position_;
    float angle_;
};

typedef std::shared_ptr<TurtlePiece> TurtlePiecePtr;