#include "player/HUD.hpp"

#include <QPainter>
#include <QPen>
#include <QFont>
#include <QImage>
#include <QPointF>
#include <QString>

#include <memory>
#include <iostream>

#include "shared/CheckersConsts.hpp"
#include "player/ImageLibrary.hpp"

HUD::HUD()
{
    m_playerColor = TurtlePieceColor::None;
    m_blackPiecesRemaining = 0u;
    m_redPiecesRemaining = 0u;
    m_blackTimeRemainingSec = 0u;
    m_redTimeRemainingSec = 0u;
    m_gameState = GameState::Connecting;
    m_winner = Winner::None;

    m_turtleFont = QFont("Times", HUD_FONT_SIZE, QFont::Bold);
    m_turtlePen = QPen(Qt::black);

    m_victoryFont = QFont("Times", VICTORY_TEXT_FONT_SIZE, QFont::Bold);
    m_victoryPen = QPen(Qt::lightGray);
    m_victoryTextCenteringRect = QRect(BOARD_LEFT, BOARD_CENTER_Y + VICTORY_TEXT_Y_OFFSET,
                                       BOARD_WIDTH, VICTORY_TEXT_HEIGHT);

    m_timerPen = QPen(Qt::black);
    m_activeTimerPen = QPen(QColor("#00FFFF"));

    m_blackRemainingTextRightingRect = QRect(HUD_BLACK_REMAINING_TEXT_X_OFFSET, HUD_REMAINING_TEXT_Y_OFFSET,
                                             HUD_REMAINING_TEXT_WIDTH, HUD_HEIGHT);
    m_redRemainingTextRightingRect = QRect(HUD_RED_REMAINING_TEXT_X_OFFSET, HUD_REMAINING_TEXT_Y_OFFSET,
                                           HUD_REMAINING_TEXT_WIDTH, HUD_HEIGHT);

    m_blackTurtleIcon = ImageLibrary::getTurtleImage(TurtlePieceColor::Black).scaled(HUD_TURTLE_ICON_HEIGHT_WIDTH, HUD_TURTLE_ICON_HEIGHT_WIDTH, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    m_blackTurtleIconPosition = QPointF(HUD_BLACK_TURTLE_ICON_X_OFFSET, HUD_TURTLE_ICON_Y_OFFSET);
    m_redTurtleIcon = ImageLibrary::getTurtleImage(TurtlePieceColor::Red).scaled(HUD_TURTLE_ICON_HEIGHT_WIDTH, HUD_TURTLE_ICON_HEIGHT_WIDTH, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    m_redTurtleIconPosition = QPointF(HUD_RED_TURTLE_ICON_X_OFFSET, HUD_TURTLE_ICON_Y_OFFSET);

    m_victoryImagePosition = QPointF(BOARD_CENTER_X, BOARD_CENTER_Y + VICTORY_IMAGE_Y_OFFSET);
    // Winner and loser images should be same size
    m_victoryImagePosition.rx() -= 0.5 * ImageLibrary::getWinnerImage().width();
    m_victoryImagePosition.ry() -= 0.5 * ImageLibrary::getWinnerImage().height();
}

void HUD::setPlayerColor(TurtlePieceColor playerColor)
{
    m_playerColor = playerColor;
}

void HUD::setPiecesRemaining(size_t blackPiecesRemaining, size_t redPiecesRemaining)
{
    m_blackPiecesRemaining = blackPiecesRemaining;
    m_redPiecesRemaining = redPiecesRemaining;
}

void HUD::enableTimers(bool usingTimers)
{
    m_usingTimers = usingTimers;
}

void HUD::setTimeRemaining(uint64_t blackTimeRemainingSec, uint64_t redTimeRemainingSec)
{
    m_lastTimestamp = std::chrono::system_clock::now();
    m_blackTimeRemainingSec = blackTimeRemainingSec;
    m_redTimeRemainingSec = redTimeRemainingSec;
}

void HUD::setGameState(GameState gameState)
{
    m_gameState = gameState;
}

void HUD::setWinner(Winner winner)
{
    m_winner = winner;
}

QString HUD::formatTimeRemaining(uint64_t timeRemaining)
{
    std::string minutes = std::to_string(timeRemaining / 60u);
    if ((timeRemaining / 60u) < 10u)
    {
        minutes = "0" + minutes;
    }
    std::string seconds = std::to_string(timeRemaining % 60u);
    if ((timeRemaining % 60u) < 10u)
    {
        seconds = "0" + seconds;
    }
    std::string combined = minutes + ":" + seconds;
    return QString(combined.c_str());
}

void HUD::paint(QPainter &painter)
{
    auto timePassedSec = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::seconds>(
                                                   std::chrono::system_clock::now() - m_lastTimestamp)
                                                   .count());

    // Fill the background
    QRgb backgroundColor = qRgb(HUD_BG_RGB[0], HUD_BG_RGB[1], HUD_BG_RGB[2]);
    painter.fillRect(0, 0, WINDOW_WIDTH, HUD_HEIGHT, backgroundColor);

    painter.setFont(m_turtleFont);
    painter.setPen(m_turtlePen);

    painter.drawImage(m_blackTurtleIconPosition, m_blackTurtleIcon);
    painter.drawImage(m_redTurtleIconPosition, m_redTurtleIcon);

    painter.drawText(m_blackRemainingTextRightingRect, Qt::AlignRight, std::to_string(m_blackPiecesRemaining).c_str());
    painter.drawText(m_redRemainingTextRightingRect, Qt::AlignRight, std::to_string(m_redPiecesRemaining).c_str());

    switch (m_gameState)
    {
    case GameState::Connecting:
    {
        painter.drawText(HUD_TEXT_X_OFFSET, HUD_HEIGHT - HUD_TEXT_Y_OFFSET, "Connecting");
    }
    break;
    case GameState::Connected:
    {
        painter.drawText(HUD_TEXT_X_OFFSET, HUD_HEIGHT - HUD_TEXT_Y_OFFSET, "Waiting for opponent to connect");
    }
    break;
    case GameState::BlackMove:
    {
        if (m_playerColor == TurtlePieceColor::Black)
        {
            painter.drawText(HUD_TEXT_X_OFFSET, HUD_HEIGHT - HUD_TEXT_Y_OFFSET, "Your move");
        }
        else
        {
            painter.drawText(HUD_TEXT_X_OFFSET, HUD_HEIGHT - HUD_TEXT_Y_OFFSET, "Opponent's move");
        }

        if (m_usingTimers)
        {
            painter.setPen(m_activeTimerPen);
            auto blackTimeRemainingSec = (m_blackTimeRemainingSec > timePassedSec) ? (m_blackTimeRemainingSec - timePassedSec) : 0u;
            painter.drawText(HUD_BLACK_TIMER_TEXT_X_OFFSET, HUD_HEIGHT - HUD_TEXT_Y_OFFSET, formatTimeRemaining(blackTimeRemainingSec));
        }
        else
        {
            painter.setPen(m_timerPen);
            painter.drawText(HUD_BLACK_TIMER_TEXT_X_OFFSET, HUD_HEIGHT - HUD_TEXT_Y_OFFSET, formatTimeRemaining(m_blackTimeRemainingSec));
        }
        painter.setPen(m_timerPen);
        painter.drawText(HUD_RED_TIMER_TEXT_X_OFFSET, HUD_HEIGHT - HUD_TEXT_Y_OFFSET, formatTimeRemaining(m_redTimeRemainingSec));
    }
    break;
    case GameState::RedMove:
    {
        if (m_playerColor == TurtlePieceColor::Red)
        {
            painter.drawText(HUD_TEXT_X_OFFSET, HUD_HEIGHT - HUD_TEXT_Y_OFFSET, "Your move");
        }
        else
        {
            painter.drawText(HUD_TEXT_X_OFFSET, HUD_HEIGHT - HUD_TEXT_Y_OFFSET, "Opponent's move");
        }

        painter.setPen(m_timerPen);
        painter.drawText(HUD_BLACK_TIMER_TEXT_X_OFFSET, HUD_HEIGHT - HUD_TEXT_Y_OFFSET, formatTimeRemaining(m_blackTimeRemainingSec));
        if (m_usingTimers)
        {
            painter.setPen(m_activeTimerPen);
            auto redTimeRemainingSec = (m_redTimeRemainingSec > timePassedSec) ? (m_redTimeRemainingSec - timePassedSec) : 0u;
            painter.drawText(HUD_RED_TIMER_TEXT_X_OFFSET, HUD_HEIGHT - HUD_TEXT_Y_OFFSET, formatTimeRemaining(redTimeRemainingSec));
        }
        else
        {
            painter.setPen(m_timerPen);
            painter.drawText(HUD_RED_TIMER_TEXT_X_OFFSET, HUD_HEIGHT - HUD_TEXT_Y_OFFSET, formatTimeRemaining(m_redTimeRemainingSec));
        }
    }
    break;
    case GameState::GameFinished:
    {
        if (m_winner == Winner::Draw)
        {
            painter.drawText(HUD_TEXT_X_OFFSET, HUD_HEIGHT - HUD_TEXT_Y_OFFSET, "Draw!");
            painter.setFont(m_victoryFont);
            painter.setPen(m_victoryPen);
            painter.drawText(m_victoryTextCenteringRect, Qt::AlignCenter, "Draw!");
            painter.drawImage(m_victoryImagePosition, ImageLibrary::getDrawImage());
        }
        else if (m_winner == Winner::Black)
        {
            if (m_playerColor == TurtlePieceColor::Black)
            {
                painter.drawText(HUD_TEXT_X_OFFSET, HUD_HEIGHT - HUD_TEXT_Y_OFFSET, "You win!");
                painter.setFont(m_victoryFont);
                painter.setPen(m_victoryPen);
                painter.drawText(m_victoryTextCenteringRect, Qt::AlignCenter, "Winner!");
                painter.drawImage(m_victoryImagePosition, ImageLibrary::getWinnerImage());
            }
            else
            {
                painter.drawText(HUD_TEXT_X_OFFSET, HUD_HEIGHT - HUD_TEXT_Y_OFFSET, "You lose!");
                painter.setFont(m_victoryFont);
                painter.setPen(m_victoryPen);
                painter.drawText(m_victoryTextCenteringRect, Qt::AlignCenter, "Loser!");
                painter.drawImage(m_victoryImagePosition, ImageLibrary::getLoserImage());
            }
        }
        else if (m_winner == Winner::Red)
        {
            if (m_playerColor == TurtlePieceColor::Red)
            {
                painter.drawText(HUD_TEXT_X_OFFSET, HUD_HEIGHT - HUD_TEXT_Y_OFFSET, "You win!");
                painter.setFont(m_victoryFont);
                painter.setPen(m_victoryPen);
                painter.drawText(m_victoryTextCenteringRect, Qt::AlignCenter, "Winner!");
                painter.drawImage(m_victoryImagePosition, ImageLibrary::getWinnerImage());
            }
            else
            {
                painter.drawText(HUD_TEXT_X_OFFSET, HUD_HEIGHT - HUD_TEXT_Y_OFFSET, "You lose!");
                painter.setFont(m_victoryFont);
                painter.setPen(m_victoryPen);
                painter.drawText(m_victoryTextCenteringRect, Qt::AlignCenter, "Loser!");
                painter.drawImage(m_victoryImagePosition, ImageLibrary::getLoserImage());
            }
        }
        else // if (m_winner == Winner::None)
        {
            // Do nothing, this shouldn't happen
        }
        painter.setFont(m_turtleFont);
        painter.setPen(m_timerPen);
        painter.drawText(HUD_BLACK_TIMER_TEXT_X_OFFSET, HUD_HEIGHT - HUD_TEXT_Y_OFFSET, formatTimeRemaining(m_blackTimeRemainingSec));
        painter.drawText(HUD_RED_TIMER_TEXT_X_OFFSET, HUD_HEIGHT - HUD_TEXT_Y_OFFSET, formatTimeRemaining(m_redTimeRemainingSec));
    }
    break;
    }
}