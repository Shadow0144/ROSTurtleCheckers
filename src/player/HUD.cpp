#include "player/HUD.hpp"

#include <QPainter>
#include <QPen>
#include <QFont>
#include <QImage>
#include <QPointF>

#include "shared/CheckersConsts.hpp"
#include "player/ImageLibrary.hpp"

HUD::HUD()
{
    m_playerColor = TurtlePieceColor::None;
    m_blackPiecesRemaining = 0u;
    m_redPiecesRemaining = 0u;
    m_gameState = GameState::Connecting;
    m_winner = Winner::None;

    m_turtleFont = QFont("Times", HUD_FONT_SIZE, QFont::Bold);
    m_turtlePen = QPen(Qt::black);

    m_victoryFont = QFont("Times", VICTORY_TEXT_FONT_SIZE, QFont::Bold);
    m_victoryPen = QPen(Qt::lightGray);
    m_victoryTextCenteringRect = QRect(BOARD_LEFT, BOARD_CENTER_Y + VICTORY_TEXT_Y_OFFSET,
                                       BOARD_WIDTH, VICTORY_TEXT_HEIGHT);

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

void HUD::setGameState(GameState gameState)
{
    m_gameState = gameState;
}

void HUD::setWinner(Winner winner)
{
    m_winner = winner;
}

void HUD::paint(QPainter &painter)
{
    // Fill the background
    QRgb backgroundColor = qRgb(HUD_BG_RGB[0], HUD_BG_RGB[1], HUD_BG_RGB[2]);
    painter.fillRect(0, 0, WINDOW_WIDTH, HUD_HEIGHT, backgroundColor);

    painter.setFont(m_turtleFont);
    painter.setPen(m_turtlePen);

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
    }
    break;
    }
}