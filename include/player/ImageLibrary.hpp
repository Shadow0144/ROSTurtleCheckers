#pragma once

#include <QImage>
#include <QVector>

#include <memory>

#include "shared/CheckersConsts.hpp"

class ImageLibrary
{
public:
    static QImage getLockImage();
    static QImage getLobbyOwnerImage();
    static QImage getKickImage();

    static QImage getTurtleImage(TurtlePieceColor turtleColor);
    static QImage getKingImage(TurtlePieceColor turtleColor);
    static QImage getMovableImage(TurtlePieceColor turtleColor);
    static QImage getHighlightImage(TurtlePieceColor turtleColor);
    static QImage getSelectImage(TurtlePieceColor turtleColor);
    static QImage getDeadImage(TurtlePieceColor turtleColor);

    static void setBlackImageIndex(size_t blackImagesIndex);
    static void setRedImageIndex(size_t redImagesIndex);

    static QImage getWinnerImage();
    static QImage getLoserImage();
    static QImage getDrawImage();

    static QImage getFlagUSImage();
    static QImage getFlagJPImage();
    static QImage getFlagDEImage();

private:
    static void createLibraryInstance();

    QImage m_lockImage;
    QImage m_lobbyOwnerImage;
    QImage m_kickImage;

    QVector<QImage> m_blackTurtleImages;
    QVector<QImage> m_redTurtleImages;
    QVector<QImage> m_randomTurtleImages;
    QVector<QImage> m_kingImages;
    QVector<QImage> m_movableImages;
    QVector<QImage> m_highlightImages;
    QVector<QImage> m_selectImages;
    QVector<QImage> m_deadImages;

    QImage m_winnerImage;
    QImage m_loserImage;
    QImage m_drawImage;

    QImage m_flagUS;
    QImage m_flagJP;
    QImage m_flagDE;

    size_t m_blackImagesIndex = 0u;
    size_t m_redImagesIndex = 0u;
};