#include "player/ImageLibrary.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp" // For getting the image directory

#include <QVector>
#include <QString>
#include <QImage>

static std::unique_ptr<ImageLibrary> s_libraryInstance;

void ImageLibrary::createLibraryInstance()
{
    s_libraryInstance = std::make_unique<ImageLibrary>();

    QVector<QString> turtlesImageNames;
    /*turtlesImageNames.append("ardent.png");
    turtlesImageNames.append("bouncy.png");
    turtlesImageNames.append("crystal.png");
    turtlesImageNames.append("dashing.png");
    turtlesImageNames.append("eloquent.png");
    turtlesImageNames.append("foxy.png");
    turtlesImageNames.append("galactic.png");
    turtlesImageNames.append("humble.png");
    turtlesImageNames.append("iron.png");
    turtlesImageNames.append("jazzy.png");*/
    turtlesImageNames.append("rolling");

    QString imagesPath =
        (ament_index_cpp::get_package_share_directory("turtle_checkers") + "/img/").c_str();
    for (int i = 0; i < turtlesImageNames.size(); ++i)
    {
        QImage bImg;
        bImg.load(imagesPath + turtlesImageNames[i] + "_black.png");
        s_libraryInstance->m_blackTurtleImages.append(bImg);
        QImage rImg;
        rImg.load(imagesPath + turtlesImageNames[i] + "_red.png");
        s_libraryInstance->m_redTurtleImages.append(rImg);
        QImage randImg;
        randImg.load(imagesPath + turtlesImageNames[i] + "_random.png");
        s_libraryInstance->m_randomTurtleImages.append(randImg);
        QImage kImg;
        kImg.load(imagesPath + turtlesImageNames[i] + "_king.png");
        s_libraryInstance->m_kingImages.append(kImg);
        QImage mImg;
        mImg.load(imagesPath + turtlesImageNames[i] + "_movable.png");
        s_libraryInstance->m_movableImages.append(mImg);
        QImage hImg;
        hImg.load(imagesPath + turtlesImageNames[i] + "_highlight.png");
        s_libraryInstance->m_highlightImages.append(hImg);
        QImage sImg;
        sImg.load(imagesPath + turtlesImageNames[i] + "_select.png");
        s_libraryInstance->m_selectImages.append(sImg);
        QImage dImg;
        dImg.load(imagesPath + turtlesImageNames[i] + "_dead.png");
        s_libraryInstance->m_deadImages.append(dImg);
    }
    
    s_libraryInstance->m_winnerImage.load(imagesPath + "winner.png");
    s_libraryInstance->m_loserImage.load(imagesPath + "loser.png");
    s_libraryInstance->m_drawImage.load(imagesPath + "draw.png");
}

QImage ImageLibrary::getTurtleImage(TurtlePieceColor turtleColor)
{
    if (!s_libraryInstance)
    {
        createLibraryInstance();
    }

    switch (turtleColor)
    {
    case TurtlePieceColor::Black:
    {
        return s_libraryInstance->m_blackTurtleImages[s_libraryInstance->m_blackImagesIndex];
    }
    break;
    case TurtlePieceColor::Red:
    {
        return s_libraryInstance->m_redTurtleImages[s_libraryInstance->m_redImagesIndex];
    }
    break;
    case TurtlePieceColor::None:
    {
        // TODO: Split into left and right halves
        return s_libraryInstance->m_randomTurtleImages[s_libraryInstance->m_blackImagesIndex];
    }
    break;
    }
    return QImage();
}

QImage ImageLibrary::getKingImage(TurtlePieceColor turtleColor)
{
    if (!s_libraryInstance)
    {
        createLibraryInstance();
    }

    switch (turtleColor)
    {
    case TurtlePieceColor::Black:
    {
        return s_libraryInstance->m_kingImages[s_libraryInstance->m_blackImagesIndex];
    }
    break;
    case TurtlePieceColor::Red:
    {
        return s_libraryInstance->m_kingImages[s_libraryInstance->m_redImagesIndex];
    }
    break;
    case TurtlePieceColor::None:
    {
        return QImage();
    }
    break;
    }
    return QImage();
}

QImage ImageLibrary::getMovableImage(TurtlePieceColor turtleColor)
{
    if (!s_libraryInstance)
    {
        createLibraryInstance();
    }

    switch (turtleColor)
    {
    case TurtlePieceColor::Black:
    {
        return s_libraryInstance->m_movableImages[s_libraryInstance->m_blackImagesIndex];
    }
    break;
    case TurtlePieceColor::Red:
    {
        return s_libraryInstance->m_movableImages[s_libraryInstance->m_redImagesIndex];
    }
    break;
    case TurtlePieceColor::None:
    {
        return QImage();
    }
    break;
    }
    return QImage();
}

QImage ImageLibrary::getHighlightImage(TurtlePieceColor turtleColor)
{
    if (!s_libraryInstance)
    {
        createLibraryInstance();
    }

    switch (turtleColor)
    {
    case TurtlePieceColor::Black:
    {
        return s_libraryInstance->m_highlightImages[s_libraryInstance->m_blackImagesIndex];
    }
    break;
    case TurtlePieceColor::Red:
    {
        return s_libraryInstance->m_highlightImages[s_libraryInstance->m_redImagesIndex];
    }
    break;
    case TurtlePieceColor::None:
    {
        return QImage();
    }
    break;
    }
    return QImage();
}

QImage ImageLibrary::getSelectImage(TurtlePieceColor turtleColor)
{
    if (!s_libraryInstance)
    {
        createLibraryInstance();
    }

    switch (turtleColor)
    {
    case TurtlePieceColor::Black:
    {
        return s_libraryInstance->m_selectImages[s_libraryInstance->m_blackImagesIndex];
    }
    break;
    case TurtlePieceColor::Red:
    {
        return s_libraryInstance->m_selectImages[s_libraryInstance->m_redImagesIndex];
    }
    break;
    case TurtlePieceColor::None:
    {
        return QImage();
    }
    break;
    }
    return QImage();
}

QImage ImageLibrary::getDeadImage(TurtlePieceColor turtleColor)
{
    if (!s_libraryInstance)
    {
        createLibraryInstance();
    }

    switch (turtleColor)
    {
    case TurtlePieceColor::Black:
    {
        return s_libraryInstance->m_deadImages[s_libraryInstance->m_blackImagesIndex];
    }
    break;
    case TurtlePieceColor::Red:
    {
        return s_libraryInstance->m_deadImages[s_libraryInstance->m_redImagesIndex];
    }
    break;
    case TurtlePieceColor::None:
    {
        return QImage();
    }
    break;
    }
    return QImage();
}

void ImageLibrary::setBlackImageIndex(size_t blackImagesIndex)
{
    if (!s_libraryInstance)
    {
        createLibraryInstance();
    }

    s_libraryInstance->m_blackImagesIndex = blackImagesIndex;
}

void ImageLibrary::setRedImageIndex(size_t redImagesIndex)
{
    if (!s_libraryInstance)
    {
        createLibraryInstance();
    }

    s_libraryInstance->m_redImagesIndex = redImagesIndex;
}

QImage ImageLibrary::getWinnerImage()
{
    if (!s_libraryInstance)
    {
        createLibraryInstance();
    }

    return s_libraryInstance->m_winnerImage;
}

QImage ImageLibrary::getLoserImage()
{
    if (!s_libraryInstance)
    {
        createLibraryInstance();
    }

    return s_libraryInstance->m_loserImage;
}

QImage ImageLibrary::getDrawImage()
{
    if (!s_libraryInstance)
    {
        createLibraryInstance();
    }

    return s_libraryInstance->m_drawImage;
}