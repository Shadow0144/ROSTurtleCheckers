#include "TurtlePieceImageLibrary.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp" // For getting the image directory

#include <QVector>
#include <QString>
#include <QImage>

static std::unique_ptr<TurtlePieceImageLibrary> s_libraryInstance;

void TurtlePieceImageLibrary::createLibraryInstance()
{
    s_libraryInstance = std::make_unique<TurtlePieceImageLibrary>();

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
        QImage kImg;
        kImg.load(imagesPath + turtlesImageNames[i] + "_king.png");
        s_libraryInstance->m_kingImages.append(kImg);
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
}

QImage TurtlePieceImageLibrary::getTurtleImage(TurtlePieceColor turtleColor)
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
        return QImage();
    }
    break;
    }
    return QImage();
}

QImage TurtlePieceImageLibrary::getKingImage(TurtlePieceColor turtleColor)
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

QImage TurtlePieceImageLibrary::getHighlightImage(TurtlePieceColor turtleColor)
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

QImage TurtlePieceImageLibrary::getSelectImage(TurtlePieceColor turtleColor)
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

QImage TurtlePieceImageLibrary::getDeadImage(TurtlePieceColor turtleColor)
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

void TurtlePieceImageLibrary::setBlackImageIndex(size_t blackImagesIndex)
{
    if (!s_libraryInstance)
    {
        createLibraryInstance();
    }

    s_libraryInstance->m_blackImagesIndex = blackImagesIndex;
}

void TurtlePieceImageLibrary::setRedImageIndex(size_t redImagesIndex)
{
    if (!s_libraryInstance)
    {
        createLibraryInstance();
    }

    s_libraryInstance->m_redImagesIndex = redImagesIndex;
}