#include "player/GameWinnerOverlayWidget.hpp"

#include <QWidget>
#include <QVBoxLayout>
#include <QLabel>
#include <QVariant>
#include <QSpacerItem>
#include <QFont>

#include <memory>

#include "shared/CheckersConsts.hpp"
#include "player/ImageLibrary.hpp"
#include "player/TranslatedQLabel.hpp"

GameWinnerOverlayWidget::GameWinnerOverlayWidget(QWidget *parent)
    : QWidget(parent)
{
    m_playerColor = TurtlePieceColor::None;
    m_winner = Winner::None;

    setProperty("dialog", QVariant(true));

    auto width = 200;
    auto height = 150;
    auto x = BOARD_CENTER_X - (width / 2);
    auto y = BOARD_CENTER_Y - (height / 2) + VICTORY_WIDGET_Y_OFFSET;
    setGeometry(x, y, width, height);

    m_victoryLayout = new QVBoxLayout();
    m_victoryLayout->setAlignment(Qt::AlignCenter);
    setLayout(m_victoryLayout);

    m_winnerImageLabel = new QLabel();
    m_winnerImageLabel->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
    auto winnerImage = QPixmap::fromImage(ImageLibrary::getWinnerImage());
    m_winnerImageLabel->setPixmap(winnerImage);
    m_winnerImageLabel->hide();
    m_victoryLayout->addWidget(m_winnerImageLabel);

    m_loserImageLabel = new QLabel();
    m_loserImageLabel->setAlignment(Qt::AlignCenter);
    auto loserImage = QPixmap::fromImage(ImageLibrary::getLoserImage());
    m_loserImageLabel->setPixmap(loserImage);
    m_loserImageLabel->hide();
    m_victoryLayout->addWidget(m_loserImageLabel);

    m_drawImageLabel = new QLabel();
    m_drawImageLabel->setAlignment(Qt::AlignCenter);
    auto drawImage = QPixmap::fromImage(ImageLibrary::getDrawImage());
    m_drawImageLabel->setPixmap(drawImage);
    m_drawImageLabel->hide();
    m_victoryLayout->addWidget(m_drawImageLabel);

    m_winnerLabel = new TranslatedQLabel();
    m_winnerLabel->setAlignment(Qt::AlignCenter);
    m_winnerLabel->setFont(QFont("Noto Sans JP", VICTORY_TEXT_FONT_SIZE, QFont::Bold));
    m_winnerLabel->setStyleSheet("color: lightGray;");
    m_victoryLayout->addWidget(m_winnerLabel);

    hide(); // Hidden by default
}

void GameWinnerOverlayWidget::reloadStrings()
{
    m_winnerLabel->reloadStrings();
}

void GameWinnerOverlayWidget::setPlayerColor(TurtlePieceColor playerColor)
{
    m_playerColor = playerColor;
}

void GameWinnerOverlayWidget::setWinner(Winner winner)
{
    m_winner = winner;

    m_winnerLabel->clear();
    m_winnerImageLabel->hide();
    m_loserImageLabel->hide();
    m_drawImageLabel->hide();

    if (m_winner == Winner::Draw)
    {
        m_winnerLabel->setText("Draw!");
        m_drawImageLabel->show();
    }
    else if (m_winner == Winner::Black)
    {
        if (m_playerColor == TurtlePieceColor::Black)
        {
            m_winnerLabel->setText("Winner!");
            m_winnerImageLabel->show();
        }
        else
        {
            m_winnerLabel->setText("Loser!");
            m_loserImageLabel->show();
        }
    }
    else if (m_winner == Winner::Red)
    {
        if (m_playerColor == TurtlePieceColor::Red)
        {
            m_winnerLabel->setText("Winner!");
            m_winnerImageLabel->show();
        }
        else
        {
            m_winnerLabel->setText("Loser!");
            m_loserImageLabel->show();
        }
    }
    else // if (m_winner == Winner::None)
    {
        // Do nothing, this shouldn't happen
    }
}