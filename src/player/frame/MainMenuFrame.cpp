#include "player/frame/MainMenuFrame.hpp"

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
#include "player/Parameters.hpp"

MainMenuFrame::MainMenuFrame(
    CheckersPlayerWindow *parentWindow)
    : QFrame(parentWindow, Qt::WindowFlags())
{
    m_playerWindow = parentWindow;

    auto mainLayout = new QVBoxLayout(this);
    mainLayout->setAlignment(Qt::AlignCenter);

    auto titleLabel = new QLabel("Turtle Checkers");
    auto titleFont = titleLabel->font();
    titleFont.setPointSize(TITLE_FONT_SIZE);
    titleLabel->setFont(titleFont);
    mainLayout->addWidget(titleLabel);

    auto playerNameLabel = new QLabel("Player name");
    mainLayout->addWidget(playerNameLabel);

    m_playerNameLineEdit = new QLineEdit();
    std::string playerNameRegex = "^[a-zA-Z0-9_]{1," + std::to_string(MAX_CHARS_NAME) + "}$";
    auto playerNameValidator = new QRegularExpressionValidator(QRegularExpression(playerNameRegex.c_str()));
    m_playerNameLineEdit->setValidator(playerNameValidator);
    m_playerNameLineEdit->setProperty("valid", false);
    connect(m_playerNameLineEdit, &QLineEdit::textChanged, this, &MainMenuFrame::validatePlayerNameText);
    mainLayout->addWidget(m_playerNameLineEdit);

    auto buttonLayout = new QHBoxLayout();

    std::string createLobbyString = "Create Lobby";
    m_createLobbyButton = new QPushButton(createLobbyString.c_str());
    m_createLobbyButton->setEnabled(false);
    connect(m_createLobbyButton, &QPushButton::released, this, &MainMenuFrame::handleCreateLobbyButton);
    buttonLayout->addWidget(m_createLobbyButton);

    std::string joinLobbyString = "Join Lobby";
    m_joinLobbyButton = new QPushButton(joinLobbyString.c_str());
    m_joinLobbyButton->setEnabled(false);
    connect(m_joinLobbyButton, &QPushButton::released, this, &MainMenuFrame::handleJoinLobbyButton);
    buttonLayout->addWidget(m_joinLobbyButton);

    std::string quitString = "Quit";
    auto quitButton = new QPushButton(quitString.c_str());
    connect(quitButton, &QPushButton::released, this, &MainMenuFrame::handleQuitButton);
    buttonLayout->addWidget(quitButton);

    mainLayout->addLayout(buttonLayout);
}

MainMenuFrame::~MainMenuFrame()
{
}

void MainMenuFrame::validatePlayerNameText(const QString &playerName)
{
    QString playerNameCopy = playerName; // Remove the const
    int pos = 0;
    QValidator::State state = m_playerNameLineEdit->validator()->validate(playerNameCopy, pos);
    if (!playerName.isEmpty() && state == QValidator::Acceptable)
    {
        m_playerNameLineEdit->setProperty("valid", true);
        m_createLobbyButton->setEnabled(true);
        m_joinLobbyButton->setEnabled(true);
        Parameters::setPlayerName(playerName.toStdString());
    }
    else if (playerName.isEmpty() || state == QValidator::Invalid)
    {
        m_playerNameLineEdit->setProperty("valid", false);
        m_createLobbyButton->setEnabled(false);
        m_joinLobbyButton->setEnabled(false);
        Parameters::setPlayerName("");
    }
    else // Intermediate
    {
        // Do nothing
    }
    // Update the style
    m_playerNameLineEdit->style()->unpolish(m_playerNameLineEdit);
    m_playerNameLineEdit->style()->polish(m_playerNameLineEdit);
    m_playerNameLineEdit->update();
}

void MainMenuFrame::handleCreateLobbyButton()
{
    m_playerWindow->moveToCreateLobbyFrame();
}

void MainMenuFrame::handleJoinLobbyButton()
{
    m_playerWindow->moveToLobbyListFrame();
}

void MainMenuFrame::handleQuitButton()
{
    m_playerWindow->close();
}