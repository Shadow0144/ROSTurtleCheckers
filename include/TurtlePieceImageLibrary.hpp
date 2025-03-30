#pragma once

#include <QImage>
#include <QVector>

#include <memory>

#include "CheckersConsts.hpp"

class TurtlePieceImageLibrary
{
public:
    static const QImage &getTurtleImage(TurtlePieceColor turtleColor);
    static const QImage &getKingImage(TurtlePieceColor turtleColor);
    static const QImage &getHighlightImage(TurtlePieceColor turtleColor);
    static const QImage &getSelectImage(TurtlePieceColor turtleColor);
    static const QImage &getDeadImage(TurtlePieceColor turtleColor);

    static void setBlackImageIndex(size_t blackImageIndex);
    static void setRedImageIndex(size_t redImageIndex);

private:
    static void createLibraryInstance();

    QVector<QImage> m_blackTurtleImages;
    size_t m_blackImageIndex = 0u;
    QVector<QImage> m_redTurtleImages;
    size_t m_redImageIndex = 0;
    QVector<QImage> m_kingImages;
    QVector<QImage> m_highlightImages;
    QVector<QImage> m_selectImages;
    QVector<QImage> m_deadImages;
};