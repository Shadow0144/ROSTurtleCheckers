#include "player/CheckersMainMenuFrame.hpp"

#include <QFrame>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPointF>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QString>
#include <QRegularExpression>
#include <QRegularExpressionValidator>
#include <QPushButton>

#include <cstdlib>
#include <ctime>
#include <functional>
#include <memory>
#include <string>
#include <chrono>
#include <vector>
#include <iostream>

#include "shared/CheckersConsts.hpp"
#include "player/CheckersPlayerWindow.hpp"
#include "player/ImageLibrary.hpp"

CheckersMainMenuFrame::CheckersMainMenuFrame(
    CheckersPlayerWindow *parentWindow)
    : QFrame(parentWindow, Qt::WindowFlags())
{
    m_playerWindow = parentWindow;
    m_playerName = "";

    setMouseTracking(true);
    std::string backgroundColorStyleString = "background-color: rgb(" +
                                             std::to_string(BG_RGB[0]) + ", " +
                                             std::to_string(BG_RGB[1]) + ", " +
                                             std::to_string(BG_RGB[2]) + ");";
    setStyleSheet(QString(backgroundColorStyleString.c_str()));

    QVBoxLayout *mainLayout = new QVBoxLayout(this);
    mainLayout->setAlignment(Qt::AlignCenter);

    std::string textColorStyleString = "color: rgb(" +
                                       std::to_string(TEXT_RGB[0]) + ", " +
                                       std::to_string(TEXT_RGB[1]) + ", " +
                                       std::to_string(TEXT_RGB[2]) + ");";
    std::string textDisabledColorStyleString = "color: rgb(" +
                                       std::to_string(TEXT_DISABLED_RGB[0]) + ", " +
                                       std::to_string(TEXT_DISABLED_RGB[1]) + ", " +
                                       std::to_string(TEXT_DISABLED_RGB[2]) + ");";
    std::string textValidColorStyleString = textColorStyleString + "border: 1px solid aqua;";
    std::string textInvalidColorStyleString = textColorStyleString + "border: 1px solid red;";
    m_playerNameLineEditDefaultStyleSheet = textValidColorStyleString.c_str();
    m_playerNameLineEditInvalidStyleSheet = textInvalidColorStyleString.c_str();

    m_buttonDefaultStyleSheet = textColorStyleString.c_str();
    m_buttonDisabledStyleSheet = textDisabledColorStyleString.c_str();

    QLabel *titleLabel = new QLabel("Turtle Checkers");
    QFont titleFont = titleLabel->font();
    titleFont.setPointSize(TITLE_FONT_SIZE);
    titleLabel->setFont(titleFont);
    titleLabel->setStyleSheet(textColorStyleString.c_str());
    mainLayout->addWidget(titleLabel);

    QLabel *playerNameLabel = new QLabel("Player name");
    playerNameLabel->setStyleSheet(textColorStyleString.c_str());
    mainLayout->addWidget(playerNameLabel);

    m_playerNameLineEdit = new QLineEdit();
    QRegularExpressionValidator *playerNameValidator = new QRegularExpressionValidator(
        QRegularExpression("[a-zA-Z0-9_]+"));
    m_playerNameLineEdit->setValidator(playerNameValidator);
    m_playerNameLineEdit->setStyleSheet(m_playerNameLineEditInvalidStyleSheet);
    connect(m_playerNameLineEdit, &QLineEdit::textChanged, this, &CheckersMainMenuFrame::validatePlayerNameText);
    mainLayout->addWidget(m_playerNameLineEdit);

    QHBoxLayout *buttonLayout = new QHBoxLayout();

    std::string createLobbyString = "Create Lobby";
    m_createLobbyButton = new QPushButton(createLobbyString.c_str());
    m_createLobbyButton->setEnabled(false);
    m_createLobbyButton->setStyleSheet(m_buttonDisabledStyleSheet);
    connect(m_createLobbyButton, &QPushButton::released, this, &CheckersMainMenuFrame::handleCreateLobbyButton);
    buttonLayout->addWidget(m_createLobbyButton);

    std::string joinLobbyString = "Join Lobby";
    m_joinLobbyButton = new QPushButton(joinLobbyString.c_str());
    m_joinLobbyButton->setEnabled(false);
    m_joinLobbyButton->setStyleSheet(m_buttonDisabledStyleSheet);
    connect(m_joinLobbyButton, &QPushButton::released, this, &CheckersMainMenuFrame::handleJoinLobbyButton);
    buttonLayout->addWidget(m_joinLobbyButton);

    std::string quitString = "Quit";
    m_quitButton = new QPushButton(quitString.c_str());
    m_quitButton->setStyleSheet(m_buttonDefaultStyleSheet);
    connect(m_quitButton, &QPushButton::released, this, &CheckersMainMenuFrame::handleQuitButton);
    buttonLayout->addWidget(m_quitButton);

    mainLayout->addLayout(buttonLayout);
}

CheckersMainMenuFrame::~CheckersMainMenuFrame()
{
}

void CheckersMainMenuFrame::setConnectedToServer(bool connected)
{
    m_connectedToServer = connected;
}

const std::string &CheckersMainMenuFrame::getPlayerName() const
{
    return m_playerName;
}

const std::string &CheckersMainMenuFrame::getLobbyName() const
{
    return m_lobbyName;
}

void CheckersMainMenuFrame::validatePlayerNameText(const QString &playerName)
{
    QString playerNameCopy = playerName; // Remove the const
    int pos = 0;
    QValidator::State state = m_playerNameLineEdit->validator()->validate(playerNameCopy, pos);
    if (!playerName.isEmpty() && state == QValidator::Acceptable)
    {
        m_playerNameLineEdit->setStyleSheet(m_playerNameLineEditDefaultStyleSheet);
        m_createLobbyButton->setEnabled(true);
        m_createLobbyButton->setStyleSheet(m_buttonDefaultStyleSheet);
        m_joinLobbyButton->setEnabled(true);
        m_joinLobbyButton->setStyleSheet(m_buttonDefaultStyleSheet);
        m_playerName = playerName.toStdString();
    }
    else if (playerName.isEmpty() || state == QValidator::Invalid)
    {
        m_playerNameLineEdit->setStyleSheet(m_playerNameLineEditInvalidStyleSheet);
        m_createLobbyButton->setEnabled(false);
        m_createLobbyButton->setStyleSheet(m_buttonDisabledStyleSheet);
        m_joinLobbyButton->setEnabled(false);
        m_joinLobbyButton->setStyleSheet(m_buttonDisabledStyleSheet);
    }
    else // Intermediate
    {
        // Do nothing
    }
}

void CheckersMainMenuFrame::handleCreateLobbyButton()
{
    m_playerWindow->createLobby(m_playerName, "Test", TurtlePieceColor::Black);
}

void CheckersMainMenuFrame::handleJoinLobbyButton()
{
    m_playerWindow->joinLobby(m_playerName, "Test", TurtlePieceColor::Red);
}

void CheckersMainMenuFrame::handleQuitButton()
{
    m_playerWindow->close();
}