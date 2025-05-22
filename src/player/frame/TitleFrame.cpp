#include "player/frame/TitleFrame.hpp"

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

TitleFrame::TitleFrame(
    CheckersPlayerWindow *parentWindow)
    : QFrame(parentWindow, Qt::WindowFlags())
{
    m_playerWindow = parentWindow;

    m_connectedToServer = false;

    auto titleLayout = new QVBoxLayout(this);
    titleLayout->setAlignment(Qt::AlignCenter);

    auto titleLabel = new QLabel("Turtle Checkers");
    auto titleFont = titleLabel->font();
    titleFont.setPointSize(TITLE_FONT_SIZE);
    titleLabel->setFont(titleFont);
    titleLayout->addWidget(titleLabel);

    m_connectingToServerLabel = new QLabel("Connecting to server...");
    m_connectingToServerLabel->setVisible(true);
    titleLayout->addWidget(m_connectingToServerLabel);

    m_connectedToServerLabel = new QLabel("Connected to server!");
    m_connectedToServerLabel->setVisible(false);
    titleLayout->addWidget(m_connectedToServerLabel);

    auto buttonLayout = new QHBoxLayout();

    std::string createAccountString = "Create New Account";
    m_createAccountButton = new QPushButton(createAccountString.c_str());
    m_createAccountButton->setEnabled(false);
    connect(m_createAccountButton, &QPushButton::released, this, &TitleFrame::handleCreateAccountButton);
    buttonLayout->addWidget(m_createAccountButton);

    std::string loginAccountString = "Login";
    m_loginAccountButton = new QPushButton(loginAccountString.c_str());
    m_loginAccountButton->setEnabled(false);
    connect(m_loginAccountButton, &QPushButton::released, this, &TitleFrame::handleLoginAccountButton);
    buttonLayout->addWidget(m_loginAccountButton);

    std::string quitString = "Quit";
    auto quitButton = new QPushButton(quitString.c_str());
    connect(quitButton, &QPushButton::released, this, &TitleFrame::handleQuitButton);
    buttonLayout->addWidget(quitButton);

    titleLayout->addLayout(buttonLayout);
}

TitleFrame::~TitleFrame()
{
}

void TitleFrame::setConnectedToServer(bool connected)
{
    m_connectedToServer = connected;
    m_createAccountButton->setEnabled(m_connectedToServer);
    m_loginAccountButton->setEnabled(m_connectedToServer);
    m_connectedToServerLabel->setVisible(m_connectedToServerLabel);
    m_connectingToServerLabel->setVisible(!m_connectedToServerLabel);
}

void TitleFrame::handleCreateAccountButton()
{
    m_playerWindow->moveToCreateAccountFrame();
}

void TitleFrame::handleLoginAccountButton()
{
    m_playerWindow->moveToLoginAccountFrame();
}

void TitleFrame::handleQuitButton()
{
    m_playerWindow->close();
}