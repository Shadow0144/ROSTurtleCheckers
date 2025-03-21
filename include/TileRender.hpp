#pragma once

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

    void toggleHighlight(bool highlight);
    
    void paint(QPainter &painter);

private:
    QPointF center_position_;

    float left;
    float top;
    float tile_width_;
    float tile_height_;

    bool highlighted;
};

typedef std::shared_ptr<TileRender> TileRenderPtr;