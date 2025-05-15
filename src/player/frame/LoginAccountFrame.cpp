#include "player/frame/LoginAccountFrame.hpp"

#include <QFrame>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPointF>
#include <QStackedLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QScrollArea>
#include <QLabel>
#include <QString>
#include <QRegularExpression>
#include <QRegularExpressionValidator>
#include <QPushButton>
#include <QRadioButton>
#include <QButtonGroup>
#include <QCheckBox>
#include <QIcon>
#include <QPixmap>
#include <QSpacerItem>
#include <QStyle>
#include <QSizePolicy>

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

LoginAccountFrame::LoginAccountFrame(
    CheckersPlayerWindow *parentWindow)
    : QFrame(parentWindow, Qt::WindowFlags())
{
    /*m_playerWindow = parentWindow;
    m_playerName = "";
    m_lobbyName = "";
    m_lobbyId = "";
    m_playerDesiredColor = TurtlePieceColor::None;
    m_playerColor = TurtlePieceColor::None;
    m_blackPlayerName = "";
    m_redPlayerName = "";
    m_blackPlayerReady = false;
    m_redPlayerReady = false;

    setMouseTracking(true);

    auto enterLobbyPasswordLayout = new QVBoxLayout(this);
    enterLobbyPasswordLayout->setAlignment(Qt::AlignCenter);

    auto enterLobbyPasswordTitleLabel = new QLabel("Turtle Checkers");
    auto titleFont = enterLobbyPasswordTitleLabel->font();
    titleFont.setPointSize(TITLE_FONT_SIZE);
    enterLobbyPasswordTitleLabel->setFont(titleFont);
    enterLobbyPasswordLayout->addWidget(enterLobbyPasswordTitleLabel);

    auto lobbyNameLayout = new QHBoxLayout();

    auto lobbyNameLabel = new QLabel(m_lobbyName.c_str());
    lobbyNameLayout->addWidget(lobbyNameLabel);

    std::string lobbyIdWithHash = "#" + m_lobbyId;
    auto lobbyIdLabel = new QLabel(lobbyIdWithHash.c_str());
    lobbyNameLayout->addWidget(lobbyIdLabel);

    enterLobbyPasswordLayout->addLayout(lobbyNameLayout);

    auto lobbyPasswordLabel = new QLabel("Lobby password");
    enterLobbyPasswordLayout->addWidget(lobbyPasswordLabel);

    m_enterLobbyPasswordLineEdit = new QLineEdit();
    m_enterLobbyPasswordLineEdit->setEchoMode(QLineEdit::Password);
    m_enterLobbyPasswordLineEdit->setProperty("in_use", false);
    connect(m_enterLobbyPasswordLineEdit, &QLineEdit::textChanged, this, &LoginAccountFrame::onEnterLobbyPasswordTextChanged);
    enterLobbyPasswordLayout->addWidget(m_enterLobbyPasswordLineEdit);

    m_passwordIncorrectLabel = new QLabel("Incorrect password");
    m_passwordIncorrectLabel->setProperty("error", true);
    auto passwordIncorrectLabelSizePolicy = m_passwordIncorrectLabel->sizePolicy();
    passwordIncorrectLabelSizePolicy.setRetainSizeWhenHidden(true);
    m_passwordIncorrectLabel->setSizePolicy(passwordIncorrectLabelSizePolicy);
    m_passwordIncorrectLabel->setVisible(false);
    enterLobbyPasswordLayout->addWidget(m_passwordIncorrectLabel);

    auto enterLobbyPasswordButtonLayout = new QHBoxLayout();

    std::string commitJoinLobbyString = "Join Lobby";
    m_commitJoinLobbyButton = new QPushButton(commitJoinLobbyString.c_str());
    m_commitJoinLobbyButton->setEnabled(false);
    connect(m_commitJoinLobbyButton, &QPushButton::released, this,
            [lobbyIndex, this]()
            { this->handleConfirmPassword(lobbyIndex); });
    enterLobbyPasswordButtonLayout->addWidget(m_commitJoinLobbyButton);

    std::string cancelJoinLobbyString = "Cancel";
    auto cancelJoinLobbyButton = new QPushButton(cancelJoinLobbyString.c_str());
    connect(cancelJoinLobbyButton, &QPushButton::released, this,
            &LoginAccountFrame::handleCancelEnterLobbyPasswordButton);
    enterLobbyPasswordButtonLayout->addWidget(cancelJoinLobbyButton);

    enterLobbyPasswordLayout->addLayout(enterLobbyPasswordButtonLayout);*/
}

LoginAccountFrame::~LoginAccountFrame()
{
}

const std::string &LoginAccountFrame::getPlayerName() const
{
    return m_playerName;
}

const std::string &LoginAccountFrame::getLobbyName() const
{
    return m_lobbyName;
}

void LoginAccountFrame::setPasswordIncorrect()
{
    //m_passwordIncorrectLabel->setVisible(true);
}

void LoginAccountFrame::onEnterLobbyPasswordTextChanged(const QString &lobbyPassword)
{
    /*m_commitJoinLobbyButton->setEnabled(!lobbyPassword.isEmpty());
    m_createLobbyPasswordLineEdit->setProperty("in_use", !lobbyPassword.isEmpty());
    // Update the style
    m_createLobbyPasswordLineEdit->style()->unpolish(m_createLobbyPasswordLineEdit);
    m_createLobbyPasswordLineEdit->style()->polish(m_createLobbyPasswordLineEdit);
    m_createLobbyPasswordLineEdit->update();*/
}

void LoginAccountFrame::handleCancelEnterLobbyPasswordButton()
{
    /*m_enterLobbyPasswordLineEdit->clear();
    m_enterLobbyPasswordLineEdit->setProperty("in_use", false);
    m_enterLobbyPasswordLineEdit->style()->unpolish(m_enterLobbyPasswordLineEdit);
    m_enterLobbyPasswordLineEdit->style()->polish(m_enterLobbyPasswordLineEdit);
    m_enterLobbyPasswordLineEdit->update();
    m_windowLayout->setCurrentIndex(JOIN_LOBBY_INDEX);*/
}

void LoginAccountFrame::handleConfirmPassword(size_t lobbyIndex)
{
    /*auto lobbyPassword = "";
    if (m_enterLobbyPasswordLineEdit)
    {
        m_enterLobbyPasswordLineEdit->text().toStdString();
    }
    m_playerWindow->joinLobby(m_playerName, m_lobbyNames[lobbyIndex], m_lobbyIds[lobbyIndex], lobbyPassword, m_playerDesiredColor);*/
}