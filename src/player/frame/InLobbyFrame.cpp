#include "player/frame/InLobbyFrame.hpp"

#include <QFrame>
#include <QWidget>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QCheckBox>
#include <QComboBox>
#include <QPixmap>
#include <QSpacerItem>

#include <cstdlib>
#include <ctime>
#include <memory>
#include <string>
#include <iostream>

#include "shared/CheckersConsts.hpp"
#include "player/Parameters.hpp"
#include "player/TitleWidget.hpp"
#include "player/DialogWidget.hpp"
#include "player/ChatBox.hpp"
#include "player/CheckersPlayerWindow.hpp"
#include "player/ImageLibrary.hpp"

InLobbyFrame::InLobbyFrame(
    CheckersPlayerWindow *parentWindow)
    : QFrame(parentWindow, Qt::WindowFlags())
{
    m_playerWindow = parentWindow;
    m_blackPlayerName = "";
    m_redPlayerName = "";
    m_blackPlayerReady = false;
    m_redPlayerReady = false;
    m_timer = std::chrono::seconds(0);

    m_gameStartTimer = new QTimer(this);
    m_gameStartTimer->setInterval(1000u); // 1000 ms (1 second)
    connect(m_gameStartTimer, &QTimer::timeout, this, [this]()
            { this->updateGameStartTimer(); });

    m_inLobbyWidget = new QWidget(this);
    auto inLobbyLayout = new QVBoxLayout();
    inLobbyLayout->setAlignment(Qt::AlignCenter);
    inLobbyLayout->setContentsMargins(IN_LOBBY_LAYOUT_MARGINS,
                                      IN_LOBBY_LAYOUT_MARGINS,
                                      IN_LOBBY_LAYOUT_MARGINS,
                                      IN_LOBBY_LAYOUT_MARGINS);
    m_inLobbyWidget->setLayout(inLobbyLayout);

    auto titleWidget = new TitleWidget();
    inLobbyLayout->addWidget(titleWidget);

    auto lobbyNameLayout = new QHBoxLayout();
    lobbyNameLayout->setAlignment(Qt::AlignLeft);

    m_lobbyNameLabel = new QLabel("");
    auto lobbyNameFont = m_lobbyNameLabel->font();
    lobbyNameFont.setPointSize(LOBBY_NAME_FONT_SIZE);
    m_lobbyNameLabel->setFont(lobbyNameFont);
    lobbyNameLayout->addWidget(m_lobbyNameLabel);

    m_lobbyIdLabel = new QLabel("");
    auto lobbyIdFont = m_lobbyIdLabel->font();
    lobbyIdFont.setPointSize(LOBBY_NAME_FONT_SIZE);
    m_lobbyIdLabel->setFont(lobbyIdFont);
    lobbyNameLayout->addWidget(m_lobbyIdLabel);

    auto spacer = new QSpacerItem(0, 0, QSizePolicy::Expanding, QSizePolicy::Expanding);
    lobbyNameLayout->addItem(spacer);

    m_gameStartTimerLabel = new QLabel();
    m_gameStartTimerLabel->setProperty("highlight", true);
    m_gameStartTimerLabel->setMargin(10);
    m_gameStartTimerLabel->setText("The match will start when both players are ready!");
    lobbyNameLayout->addWidget(m_gameStartTimerLabel);

    inLobbyLayout->addLayout(lobbyNameLayout);

    std::string readyInLobbyString = "Ready";

    std::string openString = "Open";

    // Black player layout

    QWidget *blackPlayerWidget = new QWidget();
    auto blackPlayerLayout = new QHBoxLayout();
    blackPlayerWidget->setLayout(blackPlayerLayout);
    blackPlayerWidget->setProperty("framed", true);
    blackPlayerLayout->setAlignment(Qt::AlignLeft);

    m_blackPlayerLobbyOwnerLabel = new QLabel();
    auto blackPlayerLobbyOwnerIcon = QPixmap::fromImage(ImageLibrary::getLobbyOwnerImage());
    auto scaledBlackPlayerLobbyOwnerIcon = blackPlayerLobbyOwnerIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                                                            Qt::KeepAspectRatio, Qt::SmoothTransformation);
    m_blackPlayerLobbyOwnerLabel->setPixmap(scaledBlackPlayerLobbyOwnerIcon);
    auto blackPlayerLobbyOwnerIconLabelSizePolicy = m_blackPlayerLobbyOwnerLabel->sizePolicy();
    blackPlayerLobbyOwnerIconLabelSizePolicy.setRetainSizeWhenHidden(true);
    m_blackPlayerLobbyOwnerLabel->setSizePolicy(blackPlayerLobbyOwnerIconLabelSizePolicy);
    m_blackPlayerLobbyOwnerLabel->setVisible(false);
    blackPlayerLayout->addWidget(m_blackPlayerLobbyOwnerLabel);

    m_blackReadyInLobbyCheckBox = new QCheckBox(readyInLobbyString.c_str());
    m_blackReadyInLobbyCheckBox->setEnabled(false);
    connect(m_blackReadyInLobbyCheckBox, &QCheckBox::stateChanged, this,
            &InLobbyFrame::handleBlackReadyButtonToggled);
    blackPlayerLayout->addWidget(m_blackReadyInLobbyCheckBox);

    auto blackTurtleIconLabel = new QLabel();
    auto blackTurtleIcon = QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Black));
    auto scaledBlackTurtleIcon = blackTurtleIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                                        Qt::KeepAspectRatio, Qt::SmoothTransformation);
    blackTurtleIconLabel->setPixmap(scaledBlackTurtleIcon);
    blackPlayerLayout->addWidget(blackTurtleIconLabel);

    m_blackPlayerNameLabel = new QLabel(openString.c_str());
    m_blackPlayerNameLabel->setEnabled(false);
    m_blackPlayerNameLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    blackPlayerLayout->addWidget(m_blackPlayerNameLabel);

    m_blackPlayerKickButton = new QPushButton();
    auto blackPlayerKickIcon = QPixmap::fromImage(ImageLibrary::getKickImage());
    auto scaledBlackPlayerKickIcon = blackPlayerKickIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                                                Qt::KeepAspectRatio, Qt::SmoothTransformation);
    m_blackPlayerKickButton->setIcon(scaledBlackPlayerKickIcon);
    auto blackPlayerKickButtonSizePolicy = m_blackPlayerKickButton->sizePolicy();
    blackPlayerKickButtonSizePolicy.setRetainSizeWhenHidden(true);
    m_blackPlayerKickButton->setSizePolicy(blackPlayerKickButtonSizePolicy);
    m_blackPlayerKickButton->setEnabled(false);
    m_blackPlayerKickButton->setVisible(false);
    connect(m_blackPlayerKickButton, &QPushButton::released, this,
            &InLobbyFrame::handleBlackKickButton);
    blackPlayerLayout->addWidget(m_blackPlayerKickButton);

    inLobbyLayout->addWidget(blackPlayerWidget);

    // Red player layout

    auto redPlayerWidget = new QWidget();
    auto redPlayerLayout = new QHBoxLayout();
    redPlayerWidget->setLayout(redPlayerLayout);
    redPlayerWidget->setProperty("framed", true);

    m_redPlayerLobbyOwnerLabel = new QLabel();
    auto redPlayerLobbyOwnerIcon = QPixmap::fromImage(ImageLibrary::getLobbyOwnerImage());
    auto scaledRedPlayerLobbyOwnerIcon = redPlayerLobbyOwnerIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                                                        Qt::KeepAspectRatio, Qt::SmoothTransformation);
    m_redPlayerLobbyOwnerLabel->setPixmap(scaledRedPlayerLobbyOwnerIcon);
    auto redPlayerLobbyOwnerIconLabelSizePolicy = m_redPlayerLobbyOwnerLabel->sizePolicy();
    redPlayerLobbyOwnerIconLabelSizePolicy.setRetainSizeWhenHidden(true);
    m_redPlayerLobbyOwnerLabel->setSizePolicy(redPlayerLobbyOwnerIconLabelSizePolicy);
    m_redPlayerLobbyOwnerLabel->setVisible(false);
    redPlayerLayout->addWidget(m_redPlayerLobbyOwnerLabel);

    m_redReadyInLobbyCheckBox = new QCheckBox(readyInLobbyString.c_str());
    m_redReadyInLobbyCheckBox->setEnabled(false);
    connect(m_redReadyInLobbyCheckBox, &QCheckBox::stateChanged, this,
            &InLobbyFrame::handleRedReadyButtonToggled);
    redPlayerLayout->addWidget(m_redReadyInLobbyCheckBox);

    auto redTurtleIconLabel = new QLabel();
    auto redTurtleIcon = QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Red));
    auto scaledRedTurtleIcon = redTurtleIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                                    Qt::KeepAspectRatio, Qt::SmoothTransformation);
    redTurtleIconLabel->setPixmap(scaledRedTurtleIcon);
    redPlayerLayout->addWidget(redTurtleIconLabel);

    m_redPlayerNameLabel = new QLabel(openString.c_str());
    m_redPlayerNameLabel->setEnabled(false);
    m_redPlayerNameLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    redPlayerLayout->addWidget(m_redPlayerNameLabel);

    m_redPlayerKickButton = new QPushButton();
    auto redPlayerKickIcon = QPixmap::fromImage(ImageLibrary::getKickImage());
    auto scaledRedPlayerKickIcon = redPlayerKickIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                                            Qt::KeepAspectRatio, Qt::SmoothTransformation);
    m_redPlayerKickButton->setIcon(scaledRedPlayerKickIcon);
    auto redPlayerKickButtonSizePolicy = m_redPlayerKickButton->sizePolicy();
    redPlayerKickButtonSizePolicy.setRetainSizeWhenHidden(true);
    m_redPlayerKickButton->setSizePolicy(redPlayerKickButtonSizePolicy);
    m_redPlayerKickButton->setEnabled(false);
    m_redPlayerKickButton->setVisible(false);
    connect(m_redPlayerKickButton, &QPushButton::released, this,
            &InLobbyFrame::handleRedKickButton);
    redPlayerLayout->addWidget(m_redPlayerKickButton);

    inLobbyLayout->addWidget(redPlayerWidget);

    auto timerLayout = new QHBoxLayout();

    auto timerLabel = new QLabel("Match timer:");
    timerLayout->addWidget(timerLabel);

    m_timerComboBox = new QComboBox();
    m_timerComboBox->addItem("No timer");
    m_timerComboBox->addItem("5 minutes");
    m_timerComboBox->addItem("10 minutes");
    m_timerComboBox->setEnabled(false);
    m_timerComboBox->setCurrentIndex(0);
    connect(m_timerComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            &InLobbyFrame::handleTimerIndexChanged);
    timerLayout->addWidget(m_timerComboBox);

    inLobbyLayout->addLayout(timerLayout);

    m_chatBox = new ChatBox(nullptr, CHAT_IN_LOBBY_WIDTH, CHAT_IN_LOBBY_HEIGHT, [this](const std::string &chatMessages)
                            { this->reportPlayer(chatMessages); }, [this](const std::string &chatMessage)
                            { this->sendChatMessage(chatMessage); });
    inLobbyLayout->addWidget(m_chatBox);

    auto inLobbyButtonLayout = new QHBoxLayout();

    std::string leaveLobbyInLobbyString = "Leave Lobby";
    auto leaveLobbyInLobbyButton = new QPushButton(leaveLobbyInLobbyString.c_str());
    leaveLobbyInLobbyButton->setFixedWidth(MENU_BUTTON_WIDTH);
    connect(leaveLobbyInLobbyButton, &QPushButton::released, this,
            &InLobbyFrame::handleLeaveLobbyButton);
    inLobbyButtonLayout->addWidget(leaveLobbyInLobbyButton);

    inLobbyLayout->addLayout(inLobbyButtonLayout);

    // Report player confirm dialog
    m_reportPlayerConfirmDialog = new DialogWidget(this, WINDOW_CENTER_X, WINDOW_CENTER_Y,
                                                   "Report player and leave lobby?",
                                                   "Report", {[this]()
                                                              { this->handleReportPlayerConfirmButton(); }},
                                                   "Cancel", {[this]()
                                                              { this->handleReportPlayerCancelButton(); }});
}

InLobbyFrame::~InLobbyFrame()
{
}

void InLobbyFrame::showEvent(QShowEvent *event)
{
    (void)event; // NO LINT

    m_lobbyNameLabel->setText(Parameters::getLobbyName().c_str());
    std::string lobbyIdWithHash = "#" + Parameters::getLobbyId();
    m_lobbyIdLabel->setText(lobbyIdWithHash.c_str());
}

void InLobbyFrame::hideEvent(QHideEvent *event)
{
    (void)event; // NO LINT
    m_reportPlayerConfirmDialog->hide();
    m_inLobbyWidget->setEnabled(true);
    m_gameStartTimer->stop();
}

void InLobbyFrame::updateGameStartTimer()
{
    if (m_blackPlayerReady && m_redPlayerReady)
    {
        if (m_secondsBeforeStart.count() > 0u)
        {
            m_secondsBeforeStart -= std::chrono::seconds(1u);
        }
        else
        {
            m_secondsBeforeStart = std::chrono::seconds(0u);
        }
        std::string gameStartTimerString = "The match will start in... " + std::to_string(m_secondsBeforeStart.count()) + "...";
        m_gameStartTimerLabel->setText(QString::fromStdString(gameStartTimerString));
    }
}

void InLobbyFrame::setLobbyInfo(const std::string &blackPlayerName,
                                const std::string &redPlayerName,
                                TurtlePieceColor lobbyOwnerColor,
                                bool blackPlayerReady,
                                bool redPlayerReady,
                                uint64_t timerSeconds)
{
    m_blackPlayerName = blackPlayerName;
    m_redPlayerName = redPlayerName;
    m_blackPlayerReady = blackPlayerReady;
    m_redPlayerReady = redPlayerReady;
    m_timerComboBox->setEnabled(false);
    m_timerComboBox->setCurrentIndex(0);

    auto playerName = Parameters::getPlayerName();
    if (playerName == blackPlayerName)
    {
        Parameters::setPlayerColor(TurtlePieceColor::Black);
        Parameters::setOpponentName(redPlayerName);
        Parameters::setOpponentColor(TurtlePieceColor::Red);
    }
    else if (playerName == redPlayerName)
    {
        Parameters::setPlayerColor(TurtlePieceColor::Red);
        Parameters::setOpponentName(blackPlayerName);
        Parameters::setOpponentColor(TurtlePieceColor::Black);
    }
    else
    {
        Parameters::setPlayerColor(TurtlePieceColor::None);
        Parameters::setOpponentName("");
        Parameters::setOpponentColor(TurtlePieceColor::None);
    }

    std::string openString = "Open";
    bool blackPlayerJoined = !m_blackPlayerName.empty();
    bool redPlayerJoined = !m_redPlayerName.empty();

    m_chatBox->setReportPlayerButtonEnabled(blackPlayerJoined && redPlayerJoined);

    m_blackPlayerNameLabel->setText(blackPlayerJoined ? m_blackPlayerName.c_str() : openString.c_str());
    m_blackPlayerNameLabel->setEnabled(blackPlayerJoined);
    m_redPlayerNameLabel->setText(redPlayerJoined ? m_redPlayerName.c_str() : openString.c_str());
    m_redPlayerNameLabel->setEnabled(redPlayerJoined);

    auto playerColor = Parameters::getPlayerColor();
    if (playerColor == TurtlePieceColor::Black)
    {
        m_blackReadyInLobbyCheckBox->setEnabled(true);
        if (m_blackPlayerReady)
        {
            m_blackReadyInLobbyCheckBox->setCheckState(Qt::Checked);
        }
        else
        {
            m_blackReadyInLobbyCheckBox->setCheckState(Qt::Unchecked);
        }
    }
    else
    {
        m_blackReadyInLobbyCheckBox->setEnabled(false);
        if (m_blackPlayerReady)
        {
            m_blackReadyInLobbyCheckBox->setCheckState(Qt::Checked);
        }
        else
        {
            m_blackReadyInLobbyCheckBox->setCheckState(Qt::Unchecked);
        }
    }

    if (playerColor == TurtlePieceColor::Red)
    {
        m_redReadyInLobbyCheckBox->setEnabled(true);
        if (m_redPlayerReady)
        {
            m_redReadyInLobbyCheckBox->setCheckState(Qt::Checked);
        }
        else
        {
            m_redReadyInLobbyCheckBox->setCheckState(Qt::Unchecked);
        }
    }
    else
    {
        m_redReadyInLobbyCheckBox->setEnabled(false);
        if (m_redPlayerReady)
        {
            m_redReadyInLobbyCheckBox->setCheckState(Qt::Checked);
        }
        else
        {
            m_redReadyInLobbyCheckBox->setCheckState(Qt::Unchecked);
        }
    }

    setLobbyOwnerColor(lobbyOwnerColor);
    setTimer(timerSeconds);
}

void InLobbyFrame::playerJoinedLobby(const std::string &playerName, TurtlePieceColor playerColor)
{
    if (playerName == Parameters::getPlayerName())
    {
        // Do nothing
    }

    switch (playerColor)
    {
    case TurtlePieceColor::Black:
    {
        m_blackPlayerName = playerName;
        m_blackPlayerNameLabel->setText(m_blackPlayerName.c_str());
        m_blackPlayerNameLabel->setEnabled(true);
    }
    break;
    case TurtlePieceColor::Red:
    {
        m_redPlayerName = playerName;
        m_redPlayerNameLabel->setText(m_redPlayerName.c_str());
        m_redPlayerNameLabel->setEnabled(true);
    }
    break;
    case TurtlePieceColor::None:
    {
        // Do nothing
    }
    break;
    }
    setLobbyOwnerColor(m_lobbyOwnerColor); // (Re)enable the kick button

    Parameters::setOpponentName(playerName);
    Parameters::setOpponentColor(playerColor);

    bool blackPlayerJoined = !m_blackPlayerName.empty();
    bool redPlayerJoined = !m_redPlayerName.empty();
    m_chatBox->setReportPlayerButtonEnabled(blackPlayerJoined && redPlayerJoined);
}

void InLobbyFrame::playerLeftLobby(const std::string &playerName)
{
    if (playerName == Parameters::getPlayerName())
    {
        // We've been kicked
        m_playerWindow->leaveLobby();
        m_playerWindow->moveToMainMenuFrame();
    }

    if (playerName == m_blackPlayerName)
    {
        m_blackPlayerNameLabel->setText("Open");
        m_blackPlayerNameLabel->setEnabled(false);
        m_blackReadyInLobbyCheckBox->setCheckState(Qt::Unchecked);
        m_blackPlayerName.clear();
    }
    else if (playerName == m_redPlayerName)
    {
        m_redPlayerNameLabel->setText("Open");
        m_redPlayerNameLabel->setEnabled(false);
        m_redReadyInLobbyCheckBox->setCheckState(Qt::Unchecked);
        m_redPlayerName.clear();
    }

    Parameters::setOpponentName("");
    Parameters::setOpponentColor(TurtlePieceColor::None);

    bool blackPlayerJoined = !m_blackPlayerName.empty();
    bool redPlayerJoined = !m_redPlayerName.empty();
    m_chatBox->setReportPlayerButtonEnabled(blackPlayerJoined && redPlayerJoined);
}

void InLobbyFrame::updateLobbyOwner(const std::string &lobbyOwnerPlayerName)
{
    if (lobbyOwnerPlayerName == m_blackPlayerName)
    {
        setLobbyOwnerColor(TurtlePieceColor::Black);
    }
    else if (lobbyOwnerPlayerName == m_redPlayerName)
    {
        setLobbyOwnerColor(TurtlePieceColor::Red);
    }
    else
    {
        setLobbyOwnerColor(TurtlePieceColor::None);
    }
}

void InLobbyFrame::setLobbyOwnerColor(TurtlePieceColor lobbyOwnerColor)
{
    m_lobbyOwnerColor = lobbyOwnerColor;
    switch (m_lobbyOwnerColor)
    {
    case TurtlePieceColor::Black:
    {
        m_blackPlayerLobbyOwnerLabel->setVisible(true);
        m_redPlayerLobbyOwnerLabel->setVisible(false);
        m_blackPlayerKickButton->setEnabled(false);
        m_blackPlayerKickButton->setVisible(false);
        if (Parameters::getPlayerName() == m_blackPlayerName && !m_redPlayerName.empty())
        {
            m_redPlayerKickButton->setEnabled(true);
            m_redPlayerKickButton->setVisible(true);
        }
        else
        {
            m_redPlayerKickButton->setEnabled(false);
            m_redPlayerKickButton->setVisible(false);
        }
        break;
    }
    case TurtlePieceColor::Red:
    {
        m_blackPlayerLobbyOwnerLabel->setVisible(false);
        m_redPlayerLobbyOwnerLabel->setVisible(true);
        if (Parameters::getPlayerName() == m_redPlayerName && !m_blackPlayerName.empty())
        {
            m_blackPlayerKickButton->setEnabled(true);
            m_blackPlayerKickButton->setVisible(true);
        }
        else
        {
            m_blackPlayerKickButton->setEnabled(false);
            m_blackPlayerKickButton->setVisible(false);
        }
        m_redPlayerKickButton->setEnabled(false);
        m_redPlayerKickButton->setVisible(false);
        break;
    }
    case TurtlePieceColor::None:
    {
        // This should not happen
        m_blackPlayerLobbyOwnerLabel->setVisible(false);
        m_redPlayerLobbyOwnerLabel->setVisible(false);
        m_blackPlayerKickButton->setEnabled(false);
        m_blackPlayerKickButton->setVisible(false);
        m_redPlayerKickButton->setEnabled(false);
        m_redPlayerKickButton->setVisible(false);
        break;
    }
    }
    if ((m_lobbyOwnerColor == TurtlePieceColor::Black && Parameters::getPlayerName() == m_blackPlayerName) ||
        (m_lobbyOwnerColor == TurtlePieceColor::Red && Parameters::getPlayerName() == m_redPlayerName))
    {
        m_timerComboBox->setEnabled(true);
    }
    else
    {
        m_timerComboBox->setEnabled(false);
    }
}

void InLobbyFrame::setPlayerReady(const std::string &playerName, bool ready)
{
    if (playerName == Parameters::getPlayerName())
    {
        // Do nothing
    }
    else
    {
        if (playerName == m_blackPlayerName)
        {
            m_blackReadyInLobbyCheckBox->setCheckState(ready ? Qt::Checked : Qt::Unchecked);
        }
        else if (playerName == m_redPlayerName)
        {
            m_redReadyInLobbyCheckBox->setCheckState(ready ? Qt::Checked : Qt::Unchecked);
        }
    }

    if (m_blackPlayerReady && m_redPlayerReady)
    {
        // Disable the timer so no one can change it
        // and disable booting unless someone unreadies
        m_timerComboBox->setEnabled(false);
        m_blackPlayerKickButton->setEnabled(false);
        m_redPlayerKickButton->setEnabled(false);
        m_secondsBeforeStart = MAX_SECONDS_BEFORE_START;
        std::string gameStartTimerString = "The match will start in... " + std::to_string(m_secondsBeforeStart.count()) + "...";
        m_gameStartTimerLabel->setText(QString::fromStdString(gameStartTimerString));
        m_gameStartTimer->start();
    }
    else
    {
        // Reenable the timer and boot buttons if necessary
        if ((m_lobbyOwnerColor == TurtlePieceColor::Black && Parameters::getPlayerName() == m_blackPlayerName) ||
            (m_lobbyOwnerColor == TurtlePieceColor::Red && Parameters::getPlayerName() == m_redPlayerName))
        {
            m_timerComboBox->setEnabled(true);
            m_blackPlayerKickButton->setEnabled(m_lobbyOwnerColor == TurtlePieceColor::Red);
            m_redPlayerKickButton->setEnabled(m_lobbyOwnerColor == TurtlePieceColor::Black);
        }
        m_gameStartTimerLabel->setText("The match will start when both players are ready!");
        m_gameStartTimer->stop();
    }
}

void InLobbyFrame::setTimer(uint64_t timerSeconds)
{
    std::chrono::seconds timer(timerSeconds);
    if (m_timer != timer)
    {
        int index = 0;
        if (timer == std::chrono::seconds(0))
        {
            index = 0;
        }
        else if (timer == std::chrono::minutes(5))
        {
            index = 1;
        }
        else if (timer == std::chrono::minutes(10))
        {
            index = 2;
        }
        // else - Do nothing
        m_timerComboBox->setCurrentIndex(index);
        update();
    }
}

void InLobbyFrame::clearChat()
{
    m_chatBox->clear();
}

void InLobbyFrame::addChatMessage(const std::string &playerName,
                                  TurtlePieceColor playerColor,
                                  const std::string &chatMessage,
                                  std::chrono::time_point<std::chrono::system_clock> timeStamp)
{
    m_chatBox->addMessage(playerName, playerColor, chatMessage, timeStamp);
}

void InLobbyFrame::reportPlayer(const std::string &chatMessages)
{
    m_reportingChatMessages = chatMessages;
    m_inLobbyWidget->setEnabled(false);
    m_reportPlayerConfirmDialog->show();
}

void InLobbyFrame::sendChatMessage(const std::string &chatMessage)
{
    m_playerWindow->sendChatMessage(chatMessage);
}

void InLobbyFrame::handleBlackKickButton()
{
    m_playerWindow->kickPlayer(m_blackPlayerName);
    m_blackPlayerKickButton->setEnabled(false);
    m_blackPlayerKickButton->setVisible(false);
}

void InLobbyFrame::handleRedKickButton()
{
    m_playerWindow->kickPlayer(m_redPlayerName);
    m_redPlayerKickButton->setEnabled(false);
    m_redPlayerKickButton->setVisible(false);
}

void InLobbyFrame::handleReportPlayerConfirmButton()
{
    m_playerWindow->reportPlayer(m_reportingChatMessages);
    m_reportingChatMessages.clear();
    m_playerWindow->leaveLobby();
    m_playerWindow->moveToMainMenuFrame();
}

void InLobbyFrame::handleReportPlayerCancelButton()
{
    m_inLobbyWidget->setEnabled(true);
    m_reportPlayerConfirmDialog->hide();
    m_reportingChatMessages.clear();
}

void InLobbyFrame::handleLeaveLobbyButton()
{
    m_playerWindow->leaveLobby();
    m_playerWindow->moveToMainMenuFrame();
}

void InLobbyFrame::handleBlackReadyButtonToggled(int state)
{
    m_blackPlayerReady = (state == static_cast<int>(Qt::CheckState::Checked));
    if (Parameters::getPlayerColor() == TurtlePieceColor::Black)
    {
        m_playerWindow->setReady((state == static_cast<int>(Qt::CheckState::Checked)));
    }
}

void InLobbyFrame::handleRedReadyButtonToggled(int state)
{
    m_redPlayerReady = (state == static_cast<int>(Qt::CheckState::Checked));
    if (Parameters::getPlayerColor() == TurtlePieceColor::Red)
    {
        m_playerWindow->setReady((state == static_cast<int>(Qt::CheckState::Checked)));
    }
}

void InLobbyFrame::handleTimerIndexChanged(int index)
{
    std::chrono::seconds timer;
    switch (index)
    {
    case 0:
    {
        timer = std::chrono::seconds(0);
        break;
    }
    case 1:
    {
        timer = std::chrono::minutes(5);
        break;
    }
    case 2:
    {
        timer = std::chrono::minutes(10);
        break;
    }
    default:
    {
        timer = std::chrono::seconds(0);
        break;
    }
    }
    if (timer != m_timer)
    {
        m_timer = timer;
        if (m_timerComboBox->isEnabled())
        {
            m_playerWindow->setTimer(m_timer.count());
        }
    }
}