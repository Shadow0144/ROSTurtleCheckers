#include "HUD.hpp"

#include <QPainter>
#include <QPen>
#include <QFont>
#include <QImage>
#include <QPointF>

#include "CheckersConsts.hpp"
#include "ImageLibrary.hpp"

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
    m_victoryPen = QPen(Qt::black);

    m_victoryPosition = QPointF(BOARD_CENTER_X, BOARD_CENTER_Y);
    // Winner and loser images should be same size
    m_victoryPosition.rx() -= 0.5 * ImageLibrary::getWinnerImage().width();
    m_victoryPosition.ry() -= 0.5 * ImageLibrary::getWinnerImage().height();
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
        }
        else if (m_winner == Winner::Black)
        {
            if (m_playerColor == TurtlePieceColor::Black)
            {
                painter.drawText(HUD_TEXT_X_OFFSET, HUD_HEIGHT - HUD_TEXT_Y_OFFSET, "You win!");
                painter.drawImage(m_victoryPosition, ImageLibrary::getWinnerImage());
            }
            else
            {
                painter.drawText(HUD_TEXT_X_OFFSET, HUD_HEIGHT - HUD_TEXT_Y_OFFSET, "You lose!");
                painter.drawImage(m_victoryPosition, ImageLibrary::getLoserImage());
            }
        }
        else if (m_winner == Winner::Red)
        {
            if (m_playerColor == TurtlePieceColor::Red)
            {
                painter.drawText(HUD_TEXT_X_OFFSET, HUD_HEIGHT - HUD_TEXT_Y_OFFSET, "You win!");
                painter.drawImage(m_victoryPosition, ImageLibrary::getWinnerImage());
            }
            else
            {
                painter.drawText(HUD_TEXT_X_OFFSET, HUD_HEIGHT - HUD_TEXT_Y_OFFSET, "You lose!");
                painter.drawImage(m_victoryPosition, ImageLibrary::getLoserImage());
            }
        }
        else // if (m_winner == Winner::None)
        {
            painter.drawText(HUD_TEXT_X_OFFSET, HUD_HEIGHT - HUD_TEXT_Y_OFFSET, "Draw!");
        }
    }
    break;
    }
}