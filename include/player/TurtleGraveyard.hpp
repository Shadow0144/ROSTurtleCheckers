#pragma once

#include <QWidget>

#include <memory>
#include <vector>

#include "shared/CheckersConsts.hpp"
#include "player/TurtlePieceRender.hpp"

class TurtleGraveyard : public QWidget
{
public:
    TurtleGraveyard(QWidget *parent, int x, bool left);

    void addTurtlePiece(TurtlePieceRenderPtr &turtlePieceRender);
    void clear();

    void paint(QPainter &painter);

private:
    int m_x;
    int m_y;
    bool m_left;

    int m_turtleCenterInitialY;
    int m_turtleCenterX;
    int m_turtleCenterY;
    int m_turtleCenterYStep;

    int m_turtlePieceSize;

    std::vector<TurtlePieceRenderPtr> m_turtlePieceRenders;
};

typedef std::shared_ptr<TurtleGraveyard> TurtleGraveyardPtr;