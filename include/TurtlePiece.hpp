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
        const QImage &highlight_image,
        const QImage &king_image,
        const QPointF &position,
        float angle);

    void toggleHighlight(bool highlight);

    void toggleKingship(bool king);

    void move(const QPointF &new_position);
    void paint(QPainter &painter);

private:
    std::string name_;

    QImage turtle_image_;
    QImage turtle_rotated_image_;
    QImage highlight_image_;
    QImage highlight_rotated_image_;
    QImage king_image_;
    QImage king_rotated_image_;

    QPointF position_;
    float angle_;

    bool highlighted;
    bool kinged;
};

typedef std::shared_ptr<TurtlePiece> TurtlePiecePtr;