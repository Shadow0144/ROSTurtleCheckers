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

    auto blackTurtleIconLabel = new QLabel();
    auto blackTurtleIcon = QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Black));
    auto scaledBlackTurtleIcon = blackTurtleIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                                        Qt::KeepAspectRatio, Qt::SmoothTransformation);
    blackTurtleIconLabel->setPixmap(scaledBlackTurtleIcon);
    matchLayout->addWidget(blackTurtleIconLabel);

    auto blackPlayerNameLabel = new QLabel(blackPlayerName.c_str());
    blackPlayerNameLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    blackPlayerNameLabel->setFixedWidth(MATCH_LIST_PLAYER_NAME_WIDTH);
    matchLayout->addWidget(blackPlayerNameLabel);

    auto redTurtleIconLabel = new QLabel();
    auto redTurtleIcon = QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Red));
    auto scaledRedTurtleIcon = redTurtleIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                                    Qt::KeepAspectRatio, Qt::SmoothTransformation);
    redTurtleIconLabel->setPixmap(scaledRedTurtleIcon);
    matchLayout->addWidget(redTurtleIconLabel);

    auto redPlayerNameLabel = new QLabel(redPlayerName.c_str());
    redPlayerNameLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    redPlayerNameLabel->setFixedWidth(MATCH_LIST_PLAYER_NAME_WIDTH);
    matchLayout->addWidget(redPlayerNameLabel);

    auto winnerLabel = new QLabel();
    switch (winner)
    {
    case Winner::None:
    {
        winnerLabel->setText("No winner");
    }
    break;
    case Winner::Black:
    {
        winnerLabel->setText("Black won");
    }
    break;
    case Winner::Red:
    {
        winnerLabel->setText("Red won");
    }
    break;
    case Winner::Draw:
    {
        winnerLabel->setText("Draw");
    }
    break;
    }
    matchNameLayout->addWidget(winnerLabel);
}