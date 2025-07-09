#include "player/frame/TitleFrame.hpp"

#include <QFrame>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>

#include <memory>
#include <string>
#include <iostream>

#include "shared/CheckersConsts.hpp"
#include "player/CheckersPlayerWindow.hpp"
#include "player/TitleWidget.hpp"

TitleFrame::TitleFrame(
    CheckersPlayerWindow *parentWindow)
    : QFrame(parentWindow, Qt::WindowFlags())
{
    m_playerWindow = parentWindow;

    m_connectedToServer = false;

    auto titleMainLayout = new QVBoxLayout(this);
    titleMainLayout->setAlignment(Qt::AlignCenter);

    auto titleWidget = new TitleWidget();
    titleMainLayout->addWidget(titleWidget);

    m_serverConnectionStatusLabel = new QLabel(m_connectingString);
    m_serverConnectionStatusLabel->setAlignment(Qt::AlignCenter);
    m_serverConnectionStatusLabel->setStyleSheet("padding: 10px;");
    titleMainLayout->addWidget(m_serverConnectionStatusLabel);

    auto buttonLayout = new QHBoxLayout();
    buttonLayout->setAlignment(Qt::AlignCenter);

    std::string createAccountString = "Create Account";
    m_createAccountButton = new QPushButton(createAccountString.c_str());
    m_createAccountButton->setEnabled(false);
    m_createAccountButton->setFixedWidth(MENU_BUTTON_WIDTH);
    connect(m_createAccountButton, &QPushButton::released, this, &TitleFrame::handleCreateAccountButton);
    buttonLayout->addWidget(m_createAccountButton);

    std::string logInAccountString = "Log In";
    m_logInAccountButton = new QPushButton(logInAccountString.c_str());
    m_logInAccountButton->setEnabled(false);
    m_logInAccountButton->setFixedWidth(MENU_BUTTON_WIDTH);
    connect(m_logInAccountButton, &QPushButton::released, this, &TitleFrame::handleLogInAccountButton);
    buttonLayout->addWidget(m_logInAccountButton);

    std::string quitString = "Quit";
    auto quitButton = new QPushButton(quitString.c_str());
    quitButton->setFixedWidth(MENU_BUTTON_WIDTH);
    connect(quitButton, &QPushButton::released, this, &TitleFrame::handleQuitButton);
    buttonLayout->addWidget(quitButton);

    titleMainLayout->addLayout(buttonLayout);
}

TitleFrame::~TitleFrame()
{
}

void TitleFrame::showEvent(QShowEvent *event)
{
    (void)event; // NO LINT
}

void TitleFrame::setConnectedToServer(bool connected)
{
    m_connectedToServer = connected;
    m_createAccountButton->setEnabled(m_connectedToServer);
    m_logInAccountButton->setEnabled(m_connectedToServer);
    m_serverConnectionStatusLabel->setText((connected) ? m_connectedString : m_connectingString);
}

void TitleFrame::handleCreateAccountButton()
{
    m_playerWindow->moveToCreateAccountFrame();
}

void TitleFrame::handleLogInAccountButton()
{
    m_playerWindow->moveToLogInAccountFrame();
}

void TitleFrame::handleQuitButton()
{
    m_playerWindow->close();
}