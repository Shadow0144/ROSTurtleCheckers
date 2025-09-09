#include "player/TitleWidget.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QVariant>
#include <QPushButton>
#include <QString>
#include <QStyle>

#include "shared/CheckersConsts.hpp"
#include "player/ImageLibrary.hpp"
#include "player/StringLibrary.hpp"

TitleWidget::TitleWidget()
{
    setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);

    auto titleLayout = new QHBoxLayout(this);
    titleLayout->setAlignment(Qt::AlignCenter);

    auto blackTurtleIconLabel = new QLabel();
    auto blackTurtleIcon = QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Black));
    auto scaledBlackTurtleIcon = blackTurtleIcon.scaled(TITLE_ICON_HEIGHT_WIDTH, TITLE_ICON_HEIGHT_WIDTH,
                                                        Qt::KeepAspectRatio, Qt::SmoothTransformation);
    blackTurtleIconLabel->setPixmap(scaledBlackTurtleIcon);
    titleLayout->addWidget(blackTurtleIconLabel);

    m_titleLabel = new QLabel(StringLibrary::getTranslatedString("Turtle Checkers"));
    auto titleFont = m_titleLabel->font();
    titleFont.setPointSize(TITLE_FONT_SIZE);
    m_titleLabel->setFont(titleFont);
    m_titleLabel->setAlignment(Qt::AlignCenter);
    titleLayout->addWidget(m_titleLabel);

    auto redTurtleIconLabel = new QLabel();
    auto redTurtleIcon = QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Red));
    auto scaledRedTurtleIcon = redTurtleIcon.scaled(TITLE_ICON_HEIGHT_WIDTH, TITLE_ICON_HEIGHT_WIDTH,
                                                    Qt::KeepAspectRatio, Qt::SmoothTransformation);
    redTurtleIconLabel->setPixmap(scaledRedTurtleIcon);
    titleLayout->addWidget(redTurtleIconLabel);
}

void TitleWidget::reloadStrings()
{
    m_titleLabel->setText(StringLibrary::getTranslatedString("Turtle Checkers"));
}