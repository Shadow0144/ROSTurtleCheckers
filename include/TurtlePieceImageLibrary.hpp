#pragma once

#include <QImage>
#include <QVector>

#include <memory>

#include "CheckersConsts.hpp"

class TurtlePieceImageLibrary
{
public:
    static QImage getTurtleImage(TurtlePieceColor turtleColor);
    static QImage getKingImage(TurtlePieceColor turtleColor);
    static QImage getHighlightImage(TurtlePieceColor turtleColor);
    static QImage getSelectImage(TurtlePieceColor turtleColor);
    static QImage getDeadImage(TurtlePieceColor turtleColor);

    static void setBlackImageIndex(size_t blackImagesIndex);
    static void setRedImageIndex(size_t redImagesIndex);

private:
    static void createLibraryInstance();

    QVector<QImage> m_blackTurtleImages;
    QVector<QImage> m_redTurtleImages;
    QVector<QImage> m_kingImages;
    QVector<QImage> m_highlightImages;
    QVector<QImage> m_selectImages;
    QVector<QImage> m_deadImages;

    size_t m_blackImagesIndex = 0u;
    size_t m_redImagesIndex = 0;
};