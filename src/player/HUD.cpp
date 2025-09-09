#include "player/HUD.hpp"

#include <QWidget>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QSpacerItem>
#include <QFont>

#include <memory>

#include "shared/CheckersConsts.hpp"
#include "player/ImageLibrary.hpp"
#include "player/StringLibrary.hpp"

HUD::HUD(QWidget *parent)
    : QWidget(parent)
{
    m_playerColor = TurtlePieceColor::None;
    m_blackPiecesRemaining = 0u;
    m_redPiecesRemaining = 0u;
    m_blackTimeRemainingSec = 0u;
    m_redTimeRemainingSec = 0u;
    m_gameState = GameState::Connecting;
    m_winner = Winner::None;
    m_usingTimers = false;

    setGeometry(0, 0, HUD_WIDTH, HUD_HEIGHT);

    auto hudFont = QFont("Noto Sans JP", HUD_FONT_SIZE, QFont::Bold);

    auto hudLayout = new QHBoxLayout(this);
    hudLayout->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);

    auto spacer1 = new QSpacerItem(HUD_WIDTH / 12, 0, QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);
    hudLayout->addItem(spacer1);

    m_currentTurnLabel = new QLabel();
    m_currentTurnLabel->setFont(hudFont);
    hudLayout->addWidget(m_currentTurnLabel);

    auto spacer2 = new QSpacerItem(HUD_WIDTH / 6, 0, QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);
    hudLayout->addItem(spacer2);

    m_blackPiecesRemainingLabel = new QLabel();
    m_blackPiecesRemainingLabel->setFont(hudFont);
    hudLayout->addWidget(m_blackPiecesRemainingLabel);

    auto blackTurtleIconLabel = new QLabel();
    auto blackTurtleIcon = QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Black));
    auto scaledBlackTurtleIcon = blackTurtleIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                                        Qt::KeepAspectRatio, Qt::SmoothTransformation);
    blackTurtleIconLabel->setPixmap(scaledBlackTurtleIcon);
    hudLayout->addWidget(blackTurtleIconLabel);

    m_blackTimeRemainingLabel = new QLabel();
    m_blackTimeRemainingLabel->setFont(hudFont);
    hudLayout->addWidget(m_blackTimeRemainingLabel);

    auto spacer3 = new QSpacerItem(HUD_WIDTH / 6, 0, QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);
    hudLayout->addItem(spacer3);

    m_redPiecesRemainingLabel = new QLabel();
    m_redPiecesRemainingLabel->setFont(hudFont);
    hudLayout->addWidget(m_redPiecesRemainingLabel);

    auto redTurtleIconLabel = new QLabel();
    auto redTurtleIcon = QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Red));
    auto scaledRedTurtleIcon = redTurtleIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                                    Qt::KeepAspectRatio, Qt::SmoothTransformation);
    redTurtleIconLabel->setPixmap(scaledRedTurtleIcon);
    hudLayout->addWidget(redTurtleIconLabel);

    m_redTimeRemainingLabel = new QLabel();
    m_redTimeRemainingLabel->setFont(hudFont);
    hudLayout->addWidget(m_redTimeRemainingLabel);
}

void HUD::reloadStrings()
{
    setGameState(m_gameState);
}

void HUD::setPlayerColor(TurtlePieceColor playerColor)
{
    m_playerColor = playerColor;
}

void HUD::setPiecesRemaining(size_t blackPiecesRemaining, size_t redPiecesRemaining)
{
    m_blackPiecesRemaining = blackPiecesRemaining;
    m_redPiecesRemaining = redPiecesRemaining;
    m_blackPiecesRemainingLabel->setText(QString::fromStdString(std::to_string(m_blackPiecesRemaining)));
    m_redPiecesRemainingLabel->setText(QString::fromStdString(std::to_string(m_redPiecesRemaining)));
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
    if (m_usingTimers)
    {
        m_blackTimeRemainingLabel->setText(formatTimeRemaining(m_blackTimeRemainingSec));
        m_redTimeRemainingLabel->setText(formatTimeRemaining(m_redTimeRemainingSec));
    }
    else
    {
        m_blackTimeRemainingLabel->clear();
        m_redTimeRemainingLabel->clear();
    }
}

void HUD::updateTimers()
{
    if (m_usingTimers)
    {
        auto timePassedSec = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::seconds>(
                                                       std::chrono::system_clock::now() - m_lastTimestamp)
                                                       .count());
        if (m_gameState == GameState::BlackMove)
        {
            auto blackTimeRemainingSec = (m_blackTimeRemainingSec > timePassedSec) ? (m_blackTimeRemainingSec - timePassedSec) : 0u;
            m_blackTimeRemainingLabel->setText(formatTimeRemaining(blackTimeRemainingSec));
        }
        else if (m_gameState == GameState::RedMove)
        {
            auto redTimeRemainingSec = (m_redTimeRemainingSec > timePassedSec) ? (m_redTimeRemainingSec - timePassedSec) : 0u;
            m_redTimeRemainingLabel->setText(formatTimeRemaining(redTimeRemainingSec));
        }
    }
}

void HUD::setGameState(GameState gameState)
{
    m_gameState = gameState;
    switch (m_gameState)
    {
    case GameState::Connecting:
    {
        m_currentTurnLabel->setText(StringLibrary::getTranslatedString("Connecting"));
    }
    break;
    case GameState::Connected:
    {
        m_currentTurnLabel->setText(StringLibrary::getTranslatedString("Waiting for opponent to connect"));
    }
    break;
    case GameState::BlackMove:
    {
        if (m_playerColor == TurtlePieceColor::Black)
        {
            m_currentTurnLabel->setText(StringLibrary::getTranslatedString("Your move"));
            m_currentTurnLabel->setStyleSheet("color: #00FFFF;");
        }
        else
        {
            m_currentTurnLabel->setText(StringLibrary::getTranslatedString("Opponent's move"));
            m_currentTurnLabel->setStyleSheet("color: white;");
        }

        m_blackTimeRemainingLabel->setStyleSheet("color: #00FFFF;");
        m_redTimeRemainingLabel->setStyleSheet("color: white;");
    }
    break;
    case GameState::RedMove:
    {
        if (m_playerColor == TurtlePieceColor::Red)
        {
            m_currentTurnLabel->setText(StringLibrary::getTranslatedString("Your move"));
            m_currentTurnLabel->setStyleSheet("color: #00FFFF;");
        }
        else
        {
            m_currentTurnLabel->setText(StringLibrary::getTranslatedString("Opponent's move"));
            m_redTimeRemainingLabel->setStyleSheet("color: white;");
        }

        m_blackTimeRemainingLabel->setStyleSheet("color: white;");
        m_redTimeRemainingLabel->setStyleSheet("color: #00FFFF;");
    }
    break;
    case GameState::GameFinished:
    {
        if (m_winner == Winner::Draw)
        {
            m_currentTurnLabel->setText(StringLibrary::getTranslatedString("Draw!"));
        }
        else if (m_winner == Winner::Black)
        {
            if (m_playerColor == TurtlePieceColor::Black)
            {
                m_currentTurnLabel->setText(StringLibrary::getTranslatedString("You win!"));
            }
            else
            {
                m_currentTurnLabel->setText(StringLibrary::getTranslatedString("You lose!"));
            }
        }
        else if (m_winner == Winner::Red)
        {
            if (m_playerColor == TurtlePieceColor::Red)
            {
                m_currentTurnLabel->setText(StringLibrary::getTranslatedString("You win!"));
            }
            else
            {
                m_currentTurnLabel->setText(StringLibrary::getTranslatedString("You lose!"));
            }
        }
        else // if (m_winner == Winner::None)
        {
            // Do nothing, this shouldn't happen
        }
        m_blackTimeRemainingLabel->setStyleSheet("color: white;");
        m_redTimeRemainingLabel->setStyleSheet("color: white;");
    }
    break;
    }
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