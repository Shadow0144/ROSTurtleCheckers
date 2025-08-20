#include "player/frame/TitleFrame.hpp"

#include <QFrame>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>

#include <memory>
#include <string>
#include <iostream>

#include "player/CheckersPlayerWindow.hpp"
#include "shared/CheckersConsts.hpp"
#include "player/Parameters.hpp"
#include "player/StringLibrary.hpp"
#include "player/TitleWidget.hpp"
#include "player/LanguageSelectorWidget.hpp"

TitleFrame::TitleFrame(
    CheckersPlayerWindow *parentWindow)
    : QFrame(parentWindow, Qt::WindowFlags())
{
    m_playerWindow = parentWindow;

    m_connectedToServer = false;

    auto mainLayout = new QVBoxLayout(this);
    mainLayout->setAlignment(Qt::AlignCenter);

    m_languageSelector = new LanguageSelectorWidget(this);

    m_titleWidget = new TitleWidget();
    m_titleWidget->setContentsMargins(0, 20, 0, 20);
    mainLayout->addWidget(m_titleWidget);

    auto contentWidget = new QWidget();
    auto contentLayout = new QVBoxLayout();
    contentWidget->setLayout(contentLayout);
    contentLayout->setAlignment(Qt::AlignHCenter | Qt::AlignBottom);
    mainLayout->addWidget(contentWidget);

    m_serverConnectionStatusLabel = new QLabel(StringLibrary::getTranslatedString("Connecting to server..."));
    m_serverConnectionStatusLabel->setAlignment(Qt::AlignCenter);
    m_serverConnectionStatusLabel->setContentsMargins(0, 10, 0, 0);
    contentLayout->addWidget(m_serverConnectionStatusLabel);

    auto menuButtonWidget = new QWidget();
    menuButtonWidget->setContentsMargins(0, 10, 0, 10);
    auto menuButtonLayout = new QHBoxLayout();
    menuButtonWidget->setLayout(menuButtonLayout);
    menuButtonLayout->setAlignment(Qt::AlignCenter);
    mainLayout->addWidget(menuButtonWidget);

    m_createAccountButton = new QPushButton(StringLibrary::getTranslatedString("Create Account"));
    m_createAccountButton->setEnabled(false);
    m_createAccountButton->setFixedWidth(MENU_BUTTON_WIDTH);
    connect(m_createAccountButton, &QPushButton::released, this, &TitleFrame::handleCreateAccountButton);
    menuButtonLayout->addWidget(m_createAccountButton);

    m_logInAccountButton = new QPushButton(StringLibrary::getTranslatedString("Log In"));
    m_logInAccountButton->setEnabled(false);
    m_logInAccountButton->setFixedWidth(MENU_BUTTON_WIDTH);
    connect(m_logInAccountButton, &QPushButton::released, this, &TitleFrame::handleLogInAccountButton);
    menuButtonLayout->addWidget(m_logInAccountButton);

    m_quitButton = new QPushButton(StringLibrary::getTranslatedString("Quit"));
    m_quitButton->setFixedWidth(MENU_BUTTON_WIDTH);
    connect(m_quitButton, &QPushButton::released, this, &TitleFrame::handleQuitButton);
    menuButtonLayout->addWidget(m_quitButton);
}

TitleFrame::~TitleFrame()
{
}

void TitleFrame::showEvent(QShowEvent *event)
{
    (void)event; // NO LINT

    m_languageSelector->setCurrentIndex(static_cast<int>(Parameters::getLanguage()));
    reloadStrings();
}

void TitleFrame::setConnectedToServer(bool connected)
{
    m_connectedToServer = connected;
    m_createAccountButton->setEnabled(m_connectedToServer);
    m_logInAccountButton->setEnabled(m_connectedToServer);
    m_serverConnectionStatusLabel->setText((m_connectedToServer) ? StringLibrary::getTranslatedString("Connected to server!") : StringLibrary::getTranslatedString("Connecting to server..."));
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

void TitleFrame::reloadStrings()
{
    m_titleWidget->reloadStrings();

    m_serverConnectionStatusLabel->setText((m_connectedToServer) ? StringLibrary::getTranslatedString("Connected to server!") : StringLibrary::getTranslatedString("Connecting to server..."));

    m_createAccountButton->setText(StringLibrary::getTranslatedString("Create Account"));
    m_logInAccountButton->setText(StringLibrary::getTranslatedString("Log In"));
    m_quitButton->setText(StringLibrary::getTranslatedString("Quit"));
}