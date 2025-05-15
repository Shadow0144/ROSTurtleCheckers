#include "player/frame/CreateAccountFrame.hpp"

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

CreateAccountFrame::CreateAccountFrame(
    CheckersPlayerWindow *parentWindow)
    : QFrame(parentWindow, Qt::WindowFlags())
{
    /*m_playerWindow = parentWindow;
    m_playerName = "";
    m_playerName = "";
    m_playerId = "";
    m_playerDesiredColor = TurtlePieceColor::None;
    m_playerColor = TurtlePieceColor::None;
    m_blackPlayerName = "";
    m_redPlayerName = "";
    m_blackPlayerReady = false;
    m_redPlayerReady = false;

    setMouseTracking(true);

    auto createPlayerLayout = new QVBoxLayout(this);
    createPlayerLayout->setAlignment(Qt::AlignCenter);

    auto createPlayerTitleLabel = new QLabel("Turtle Checkers");
    auto titleFont = createPlayerTitleLabel->font();
    titleFont.setPointSize(TITLE_FONT_SIZE);
    createPlayerTitleLabel->setFont(titleFont);
    createPlayerLayout->addWidget(createPlayerTitleLabel);

    auto playerNameLabel = new QLabel("Player name");
    createPlayerLayout->addWidget(playerNameLabel);

    m_playerNameLineEdit = new QLineEdit();
    std::string playerNameRegex = "^[a-zA-Z][a-zA-Z0-9_]{0," + std::to_string(MAX_CHARS_NAME - 1) + "}$";
    auto playerNameValidator = new QRegularExpressionValidator(QRegularExpression(playerNameRegex.c_str()));
    m_playerNameLineEdit->setValidator(playerNameValidator);
    m_playerNameLineEdit->setProperty("valid", false);
    connect(m_playerNameLineEdit, &QLineEdit::textChanged, this, &CreateAccountFrame::validatePlayerNameText);
    createPlayerLayout->addWidget(m_playerNameLineEdit);

    auto playerPasswordLabel = new QLabel("Player password");
    createPlayerLayout->addWidget(playerPasswordLabel);

    m_createPlayerPasswordLineEdit = new QLineEdit();
    m_createPlayerPasswordLineEdit->setEchoMode(QLineEdit::Password);
    m_createPlayerPasswordLineEdit->setProperty("in_use", false);
    connect(m_createPlayerPasswordLineEdit, &QLineEdit::textChanged, this, &CreateAccountFrame::onCreatePlayerPasswordTextChanged);
    createPlayerLayout->addWidget(m_createPlayerPasswordLineEdit);

    auto createPlayerDesiredColorLayout = new QHBoxLayout();
    createPlayerDesiredColorLayout->setAlignment(Qt::AlignCenter);

    m_createPlayerBlackRadioButton = new QRadioButton();
    m_createPlayerRandomRadioButton = new QRadioButton();
    m_createPlayerRedRadioButton = new QRadioButton();

    m_createPlayerBlackRadioButton->setIcon(QIcon(QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Black))));
    m_createPlayerRandomRadioButton->setIcon(QIcon(QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::None))));
    m_createPlayerRedRadioButton->setIcon(QIcon(QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Red))));

    connect(m_createPlayerBlackRadioButton, &QRadioButton::toggled, this, &CreateAccountFrame::onBlackTurtleToggled);
    connect(m_createPlayerRandomRadioButton, &QRadioButton::toggled, this, &CreateAccountFrame::onRandomTurtleToggled);
    connect(m_createPlayerRedRadioButton, &QRadioButton::toggled, this, &CreateAccountFrame::onRedTurtleToggled);

    createPlayerDesiredColorLayout->addWidget(m_createPlayerBlackRadioButton);
    createPlayerDesiredColorLayout->addWidget(m_createPlayerRandomRadioButton);
    createPlayerDesiredColorLayout->addWidget(m_createPlayerRedRadioButton);

    createPlayerLayout->addLayout(createPlayerDesiredColorLayout);

    auto createPlayerButtonLayout = new QHBoxLayout();

    std::string commitCreatePlayerString = "Create Player";
    m_commitCreatePlayerButton = new QPushButton(commitCreatePlayerString.c_str());
    m_commitCreatePlayerButton->setEnabled(false);
    connect(m_commitCreatePlayerButton, &QPushButton::released, this,
            &CreateAccountFrame::handleCommitCreatePlayerButton);
    createPlayerButtonLayout->addWidget(m_commitCreatePlayerButton);

    std::string cancelCreatePlayerString = "Cancel";
    auto cancelCreatePlayerButton = new QPushButton(cancelCreatePlayerString.c_str());
    connect(cancelCreatePlayerButton, &QPushButton::released, this,
            &CreateAccountFrame::handleCancelCreatePlayerButton);
    createPlayerButtonLayout->addWidget(cancelCreatePlayerButton);

    createPlayerLayout->addLayout(createPlayerButtonLayout);*/
}

CreateAccountFrame::~CreateAccountFrame()
{
}

void CreateAccountFrame::validatePlayerNameText(const QString &playerName)
{
    /*QString playerNameCopy = playerName; // Remove the const
    int pos = 0;
    QValidator::State state = m_playerNameLineEdit->validator()->validate(playerNameCopy, pos);
    if (!playerName.isEmpty() && state == QValidator::Acceptable)
    {
        m_playerNameLineEdit->setProperty("valid", true);
        m_commitCreatePlayerButton->setEnabled(true);
        m_playerName = playerName.toStdString();
    }
    else if (playerName.isEmpty() || state == QValidator::Invalid)
    {
        m_playerNameLineEdit->setProperty("valid", false);
        m_commitCreatePlayerButton->setEnabled(false);
        m_playerName = "";
    }
    else // Intermediate
    {
        // Do nothing
    }
    // Update the style
    m_playerNameLineEdit->style()->unpolish(m_playerNameLineEdit);
    m_playerNameLineEdit->style()->polish(m_playerNameLineEdit);
    m_playerNameLineEdit->update();*/
}

void CreateAccountFrame::onCreatePlayerPasswordTextChanged(const QString &playerPassword)
{
    /*m_createPlayerPasswordLineEdit->setProperty("in_use", !playerPassword.isEmpty());
    // Update the style
    m_createPlayerPasswordLineEdit->style()->unpolish(m_createPlayerPasswordLineEdit);
    m_createPlayerPasswordLineEdit->style()->polish(m_createPlayerPasswordLineEdit);
    m_createPlayerPasswordLineEdit->update();*/
}

void CreateAccountFrame::handleCancelCreatePlayerButton()
{
    /*m_playerName = "";
    m_playerId = "";
    m_playerDesiredColor = TurtlePieceColor::None;
    m_playerNameLineEdit->clear();
    m_playerNameLineEdit->setProperty("valid", false);
    m_playerNameLineEdit->style()->unpolish(m_playerNameLineEdit);
    m_playerNameLineEdit->style()->polish(m_playerNameLineEdit);
    m_playerNameLineEdit->update();
    m_createPlayerPasswordLineEdit->clear();
    m_createPlayerPasswordLineEdit->setProperty("in_use", false);
    m_createPlayerPasswordLineEdit->style()->unpolish(m_createPlayerPasswordLineEdit);
    m_createPlayerPasswordLineEdit->style()->polish(m_createPlayerPasswordLineEdit);
    m_createPlayerPasswordLineEdit->update();
    m_commitCreatePlayerButton->setEnabled(false);
    m_playerNameLineEdit->setText(m_playerName.c_str());
    m_windowLayout->setCurrentIndex(MAIN_MENU_INDEX);*/
}

void CreateAccountFrame::handleCommitCreatePlayerButton()
{
    /*auto playerPassword = m_createPlayerPasswordLineEdit->text().toStdString();
    m_playerWindow->createPlayer(m_playerName, m_playerName, playerPassword, m_playerDesiredColor);*/
}