#include "player/MatchDetailsWidget.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QVariant>
#include <QPushButton>
#include <QString>
#include <QStyle>

#include <string>
#include <vector>
#include <functional>

#include "shared/CheckersConsts.hpp"
#include "player/ImageLibrary.hpp"

MatchDetailsWidget::MatchDetailsWidget(QWidget *parent,
                                       const std::string &playerName,
                                       const std::string &lobbyNameId,
                                       const std::string &blackPlayerName,
                                       const std::string &redPlayerName,
                                       Winner winner)
    : QWidget(parent)
{
    setProperty("details", QVariant(true));
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    // Update the style
    style()->unpolish(this);
    style()->polish(this);
    update();

    auto matchLayout = new QHBoxLayout(this);

    auto matchNameLayout = new QHBoxLayout();
    auto matchIdLabel = new QLabel(lobbyNameId.c_str());
    matchIdLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    matchIdLabel->setFixedWidth(MATCH_LIST_LOBBY_NAME_WIDTH);
    matchNameLayout->addWidget(matchIdLabel);
    matchLayout->addLayout(matchNameLayout);

    auto winIcon = QPixmap::fromImage(ImageLibrary::getWinnerImage());
    auto scaledWinIcon = winIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                        Qt::KeepAspectRatio, Qt::SmoothTransformation);

    auto loseIcon = QPixmap::fromImage(ImageLibrary::getLoserImage());
    auto scaledLoseIcon = loseIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                          Qt::KeepAspectRatio, Qt::SmoothTransformation);

    auto drawIcon = QPixmap::fromImage(ImageLibrary::getDrawImage());
    auto scaledDrawIcon = drawIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                          Qt::KeepAspectRatio, Qt::SmoothTransformation);

    auto winnerLayout = new QHBoxLayout();
    auto winnerIconLabel = new QLabel();
    auto winnerLabel = new QLabel();
    winnerLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    winnerLabel->setFixedWidth(MATCH_LIST_PLAYER_NAME_WIDTH);

    switch (winner)
    {
    case Winner::None:
    {
        winnerLabel->setText("No winner");
    }
    break;
    case Winner::Black:
    {
        winnerLabel->setText(QString::fromStdString(blackPlayerName + " won"));
        if (playerName == blackPlayerName)
        {
            winnerIconLabel->setPixmap(scaledWinIcon);
        }
        else if (playerName == redPlayerName)
        {
            winnerIconLabel->setPixmap(scaledLoseIcon);
        }
    }
    break;
    case Winner::Red:
    {
        winnerLabel->setText(QString::fromStdString(redPlayerName + " won"));
        if (playerName == blackPlayerName)
        {
            winnerIconLabel->setPixmap(scaledLoseIcon);
        }
        else if (playerName == redPlayerName)
        {
            winnerIconLabel->setPixmap(scaledWinIcon);
        }
    }
    break;
    case Winner::Draw:
    {
        winnerLabel->setText("Draw");
        if (playerName == blackPlayerName)
        {
            winnerIconLabel->setPixmap(scaledDrawIcon);
        }
        else if (playerName == redPlayerName)
        {
            winnerIconLabel->setPixmap(scaledDrawIcon);
        }
    }
    break;
    }

    winnerLayout->addWidget(winnerIconLabel);
    winnerLayout->addWidget(winnerLabel);
    matchLayout->addLayout(winnerLayout);

    auto blackPlayerLayout = new QHBoxLayout();

    auto blackTurtleIconLabel = new QLabel();
    auto blackTurtleIcon = QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Black));
    auto scaledBlackTurtleIcon = blackTurtleIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                                        Qt::KeepAspectRatio, Qt::SmoothTransformation);
    blackTurtleIconLabel->setPixmap(scaledBlackTurtleIcon);
    blackPlayerLayout->addWidget(blackTurtleIconLabel);

    auto blackPlayerNameLabel = new QLabel(blackPlayerName.c_str());
    blackPlayerNameLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    blackPlayerNameLabel->setFixedWidth(MATCH_LIST_PLAYER_NAME_WIDTH);
    blackPlayerLayout->addWidget(blackPlayerNameLabel);

    matchLayout->addLayout(blackPlayerLayout);

    auto redPlayerLayout = new QHBoxLayout();

    auto redTurtleIconLabel = new QLabel();
    auto redTurtleIcon = QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Red));
    auto scaledRedTurtleIcon = redTurtleIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                                    Qt::KeepAspectRatio, Qt::SmoothTransformation);
    redTurtleIconLabel->setPixmap(scaledRedTurtleIcon);
    redPlayerLayout->addWidget(redTurtleIconLabel);

    auto redPlayerNameLabel = new QLabel(redPlayerName.c_str());
    redPlayerNameLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    redPlayerNameLabel->setFixedWidth(MATCH_LIST_PLAYER_NAME_WIDTH);
    redPlayerLayout->addWidget(redPlayerNameLabel);

    matchLayout->addLayout(redPlayerLayout);
}