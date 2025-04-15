#include "player/CheckersMainMenuFrame.hpp"

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
    m_lobbyName = "";
    m_lobbyId = "";
    m_playerDesiredColor = TurtlePieceColor::None;
    m_playerColor = TurtlePieceColor::None;
    m_blackPlayerName = "";
    m_redPlayerName = "";
    m_blackPlayerReady = false;
    m_redPlayerReady = false;

    setMouseTracking(true);

    m_windowLayout = new QStackedLayout(this);

    // Create and add the screens to the layout
    m_windowLayout->insertWidget(MAIN_MENU_INDEX, createMainMenuScreen());
    m_windowLayout->insertWidget(CREATE_LOBBY_INDEX, createCreateLobbyScreen());
    m_windowLayout->insertWidget(JOIN_LOBBY_INDEX, new QWidget());
    m_windowLayout->insertWidget(IN_LOBBY_INDEX, new QWidget());
    m_windowLayout->setCurrentIndex(MAIN_MENU_INDEX);
}

CheckersMainMenuFrame::~CheckersMainMenuFrame()
{
}

QWidget *CheckersMainMenuFrame::createMainMenuScreen()
{
    auto mainLayoutWidget = new QWidget();
    auto mainLayout = new QVBoxLayout(mainLayoutWidget);
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
    connect(m_playerNameLineEdit, &QLineEdit::textChanged, this, &CheckersMainMenuFrame::validatePlayerNameText);
    mainLayout->addWidget(m_playerNameLineEdit);

    auto buttonLayout = new QHBoxLayout();

    std::string createLobbyString = "Create Lobby";
    m_createLobbyButton = new QPushButton(createLobbyString.c_str());
    m_createLobbyButton->setEnabled(false);
    connect(m_createLobbyButton, &QPushButton::released, this, &CheckersMainMenuFrame::handleCreateLobbyButton);
    buttonLayout->addWidget(m_createLobbyButton);

    std::string joinLobbyString = "Join Lobby";
    m_joinLobbyButton = new QPushButton(joinLobbyString.c_str());
    m_joinLobbyButton->setEnabled(false);
    connect(m_joinLobbyButton, &QPushButton::released, this, &CheckersMainMenuFrame::handleJoinLobbyButton);
    buttonLayout->addWidget(m_joinLobbyButton);

    std::string quitString = "Quit";
    auto quitButton = new QPushButton(quitString.c_str());
    connect(quitButton, &QPushButton::released, this, &CheckersMainMenuFrame::handleQuitButton);
    buttonLayout->addWidget(quitButton);

    mainLayout->addLayout(buttonLayout);

    return mainLayoutWidget;
}

QWidget *CheckersMainMenuFrame::createCreateLobbyScreen()
{
    auto createLobbyLayoutWidget = new QWidget();
    auto createLobbyLayout = new QVBoxLayout(createLobbyLayoutWidget);
    createLobbyLayout->setAlignment(Qt::AlignCenter);

    auto createLobbyTitleLabel = new QLabel("Turtle Checkers");
    auto titleFont = createLobbyTitleLabel->font();
    titleFont.setPointSize(TITLE_FONT_SIZE);
    createLobbyTitleLabel->setFont(titleFont);
    createLobbyLayout->addWidget(createLobbyTitleLabel);

    auto lobbyNameLabel = new QLabel("Lobby name");
    createLobbyLayout->addWidget(lobbyNameLabel);

    m_lobbyNameLineEdit = new QLineEdit();
    std::string lobbyNameRegex = "^[a-zA-Z][a-zA-Z0-9_]{0," + std::to_string(MAX_CHARS_NAME - 1) + "}$";
    auto lobbyNameValidator = new QRegularExpressionValidator(QRegularExpression(lobbyNameRegex.c_str()));
    m_lobbyNameLineEdit->setValidator(lobbyNameValidator);
    m_lobbyNameLineEdit->setProperty("valid", false);
    connect(m_lobbyNameLineEdit, &QLineEdit::textChanged, this, &CheckersMainMenuFrame::validatelobbyNameText);
    createLobbyLayout->addWidget(m_lobbyNameLineEdit);

    auto createLobbyDesiredColorLayout = new QHBoxLayout();
    createLobbyDesiredColorLayout->setAlignment(Qt::AlignCenter);

    m_createLobbyBlackRadioButton = new QRadioButton();
    m_createLobbyRandomRadioButton = new QRadioButton();
    m_createLobbyRedRadioButton = new QRadioButton();

    m_createLobbyBlackRadioButton->setIcon(QIcon(QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Black))));
    m_createLobbyRandomRadioButton->setIcon(QIcon(QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::None))));
    m_createLobbyRedRadioButton->setIcon(QIcon(QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Red))));

    connect(m_createLobbyBlackRadioButton, &QRadioButton::toggled, this, &CheckersMainMenuFrame::onBlackTurtleToggled);
    connect(m_createLobbyRandomRadioButton, &QRadioButton::toggled, this, &CheckersMainMenuFrame::onRandomTurtleToggled);
    connect(m_createLobbyRedRadioButton, &QRadioButton::toggled, this, &CheckersMainMenuFrame::onRedTurtleToggled);

    createLobbyDesiredColorLayout->addWidget(m_createLobbyBlackRadioButton);
    createLobbyDesiredColorLayout->addWidget(m_createLobbyRandomRadioButton);
    createLobbyDesiredColorLayout->addWidget(m_createLobbyRedRadioButton);

    createLobbyLayout->addLayout(createLobbyDesiredColorLayout);

    auto createLobbyButtonLayout = new QHBoxLayout();

    std::string commitCreateLobbyString = "Create Lobby";
    m_commitCreateLobbyButton = new QPushButton(commitCreateLobbyString.c_str());
    m_commitCreateLobbyButton->setEnabled(false);
    connect(m_commitCreateLobbyButton, &QPushButton::released, this,
            &CheckersMainMenuFrame::handleCommitCreateLobbyButton);
    createLobbyButtonLayout->addWidget(m_commitCreateLobbyButton);

    std::string cancelCreateLobbyString = "Cancel";
    auto cancelCreateLobbyButton = new QPushButton(cancelCreateLobbyString.c_str());
    connect(cancelCreateLobbyButton, &QPushButton::released, this,
            &CheckersMainMenuFrame::handleCancelCreateLobbyButton);
    createLobbyButtonLayout->addWidget(cancelCreateLobbyButton);

    createLobbyLayout->addLayout(createLobbyButtonLayout);

    return createLobbyLayoutWidget;
}

QWidget *CheckersMainMenuFrame::createJoinLobbyScreen()
{
    auto joinLobbyLayoutWidget = new QWidget();
    auto joinLobbyLayout = new QVBoxLayout(joinLobbyLayoutWidget);
    joinLobbyLayout->setAlignment(Qt::AlignCenter);

    auto joinLobbyTitleLabel = new QLabel("Turtle Checkers");
    auto titleFont = joinLobbyTitleLabel->font();
    titleFont.setPointSize(TITLE_FONT_SIZE);
    joinLobbyTitleLabel->setFont(titleFont);
    joinLobbyLayout->addWidget(joinLobbyTitleLabel);

    auto lobbiesLabel = new QLabel("Lobbies:");
    joinLobbyLayout->addWidget(lobbiesLabel);

    auto lobbyListScrollArea = new QScrollArea();
    lobbyListScrollArea->setFixedSize(LOBBY_SCROLL_W, LOBBY_SCROLL_H);
    lobbyListScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    lobbyListScrollArea->setObjectName("LobbyListScrollArea");

    auto lobbyListLayoutWidget = new QWidget();
    lobbyListLayoutWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    auto lobbyListLayout = new QVBoxLayout(lobbyListLayoutWidget);

    const auto numLobbies = m_lobbyNames.size();
    if (m_lobbyIds.size() != numLobbies)
    {
        std::cerr << "Lobby name vector does not match size of lobby id vector" << std::endl;
        return nullptr;
    }
    for (size_t i = 0u; i < numLobbies; i++)
    {
        auto lobbyLayoutWidget = new QWidget();
        lobbyLayoutWidget->setProperty("lobby", true);
        lobbyLayoutWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        // Update the style
        lobbyLayoutWidget->style()->unpolish(lobbyLayoutWidget);
        lobbyLayoutWidget->style()->polish(lobbyLayoutWidget);
        lobbyLayoutWidget->update();

        auto lobbyLayout = new QHBoxLayout(lobbyLayoutWidget);

        auto lobbyNameLayout = new QHBoxLayout();

        auto lobbyNameLabel = new QLabel(m_lobbyNames[i].c_str());
        lobbyNameLayout->addWidget(lobbyNameLabel);

        std::string lobbyIdWithHash = "#" + m_lobbyIds[i];
        auto lobbyIdLabel = new QLabel(lobbyIdWithHash.c_str());
        lobbyNameLayout->addWidget(lobbyIdLabel);

        lobbyLayout->addLayout(lobbyNameLayout);

        auto blackTurtleIconLabel = new QLabel();
        auto blackTurtleIcon = QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Black));
        auto scaledBlackTurtleIcon = blackTurtleIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                                            Qt::KeepAspectRatio, Qt::SmoothTransformation);
        blackTurtleIconLabel->setPixmap(scaledBlackTurtleIcon);
        lobbyLayout->addWidget(blackTurtleIconLabel);

        std::string openString = "Open";
        bool blackPlayerJoined = !m_blackPlayerNames[i].empty();
        bool redPlayerJoined = !m_redPlayerNames[i].empty();

        auto blackPlayerNameLabel = new QLabel(blackPlayerJoined ? m_blackPlayerNames[i].c_str() : openString.c_str());
        blackPlayerNameLabel->setEnabled(blackPlayerJoined);
        lobbyLayout->addWidget(blackPlayerNameLabel);

        auto redTurtleIconLabel = new QLabel();
        auto redTurtleIcon = QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Red));
        auto scaledRedTurtleIcon = redTurtleIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                                        Qt::KeepAspectRatio, Qt::SmoothTransformation);
        redTurtleIconLabel->setPixmap(scaledRedTurtleIcon);
        lobbyLayout->addWidget(redTurtleIconLabel);

        auto redPlayerNameLabel = new QLabel(redPlayerJoined ? m_redPlayerNames[i].c_str() : openString.c_str());
        redPlayerNameLabel->setEnabled(redPlayerJoined);
        lobbyLayout->addWidget(redPlayerNameLabel);

        std::string joinLobbyString = "Join";
        auto joinLobbyButton = new QPushButton(joinLobbyString.c_str());
        if (!blackPlayerJoined || !redPlayerJoined)
        {
            joinLobbyButton->setEnabled(true);
        }
        else
        {
            // Lobby is full
            joinLobbyButton->setEnabled(false);
        }
        connect(joinLobbyButton, &QPushButton::released, this,
                [i, this]()
                { this->handleCommitJoinLobbyButton(i); });
        lobbyLayout->addWidget(joinLobbyButton);

        lobbyListLayout->addWidget(lobbyLayoutWidget);
    }

    auto spacer = new QSpacerItem(0, 0, QSizePolicy::Preferred, QSizePolicy::Expanding);
    lobbyListLayout->addItem(spacer);

    lobbyListScrollArea->setWidget(lobbyListLayoutWidget);

    joinLobbyLayout->addWidget(lobbyListScrollArea);

    auto joinLobbyDesiredColorLayout = new QHBoxLayout();
    joinLobbyDesiredColorLayout->setAlignment(Qt::AlignCenter);

    m_joinLobbyBlackRadioButton = new QRadioButton();
    m_joinLobbyRandomRadioButton = new QRadioButton();
    m_joinLobbyRedRadioButton = new QRadioButton();

    m_joinLobbyBlackRadioButton->setIcon(QIcon(QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Black))));
    m_joinLobbyRandomRadioButton->setIcon(QIcon(QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::None))));
    m_joinLobbyRedRadioButton->setIcon(QIcon(QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Red))));

    switch (m_playerDesiredColor)
    {
    case TurtlePieceColor::Black:
    {
        m_joinLobbyBlackRadioButton->setChecked(true);
    }
    break;
    case TurtlePieceColor::None:
    {
        m_joinLobbyRandomRadioButton->setChecked(true);
    }
    break;
    case TurtlePieceColor::Red:
    {
        m_joinLobbyRedRadioButton->setChecked(true);
    }
    break;
    }

    connect(m_joinLobbyBlackRadioButton, &QRadioButton::toggled, this, &CheckersMainMenuFrame::onBlackTurtleToggled);
    connect(m_joinLobbyRandomRadioButton, &QRadioButton::toggled, this, &CheckersMainMenuFrame::onRandomTurtleToggled);
    connect(m_joinLobbyRedRadioButton, &QRadioButton::toggled, this, &CheckersMainMenuFrame::onRedTurtleToggled);

    joinLobbyDesiredColorLayout->addWidget(m_joinLobbyBlackRadioButton);
    joinLobbyDesiredColorLayout->addWidget(m_joinLobbyRandomRadioButton);
    joinLobbyDesiredColorLayout->addWidget(m_joinLobbyRedRadioButton);

    joinLobbyLayout->addLayout(joinLobbyDesiredColorLayout);

    auto joinLobbyButtonLayout = new QHBoxLayout();

    std::string refreshjoinLobbyString = "Refresh";
    auto refreshJoinLobbyButton = new QPushButton(refreshjoinLobbyString.c_str());
    connect(refreshJoinLobbyButton, &QPushButton::released, this,
            &CheckersMainMenuFrame::handleRefreshJoinLobbyButton);
    joinLobbyButtonLayout->addWidget(refreshJoinLobbyButton);

    std::string canceljoinLobbyString = "Cancel";
    auto cancelJoinLobbyButton = new QPushButton(canceljoinLobbyString.c_str());
    connect(cancelJoinLobbyButton, &QPushButton::released, this,
            &CheckersMainMenuFrame::handleCancelJoinLobbyButton);
    joinLobbyButtonLayout->addWidget(cancelJoinLobbyButton);

    joinLobbyLayout->addLayout(joinLobbyButtonLayout);

    return joinLobbyLayoutWidget;
}

QWidget *CheckersMainMenuFrame::createInLobbyScreen()
{
    auto inLobbyLayoutWidget = new QWidget();
    auto inLobbyLayout = new QVBoxLayout(inLobbyLayoutWidget);
    inLobbyLayout->setAlignment(Qt::AlignCenter);

    auto inLobbyTitleLabel = new QLabel("Turtle Checkers");
    auto titleFont = inLobbyTitleLabel->font();
    titleFont.setPointSize(TITLE_FONT_SIZE);
    inLobbyTitleLabel->setFont(titleFont);
    inLobbyLayout->addWidget(inLobbyTitleLabel);

    auto lobbyNameLayout = new QHBoxLayout();

    auto lobbyNameLabel = new QLabel(m_lobbyName.c_str());
    lobbyNameLayout->addWidget(lobbyNameLabel);

    std::string lobbyIdWithHash = "#" + m_lobbyId;
    auto lobbyIdLabel = new QLabel(lobbyIdWithHash.c_str());
    lobbyNameLayout->addWidget(lobbyIdLabel);

    inLobbyLayout->addLayout(lobbyNameLayout);

    std::string readyInLobbyString = "Ready";

    auto blackPlayerLayout = new QHBoxLayout();

    m_blackReadyInLobbyCheckBox = new QCheckBox(readyInLobbyString.c_str());
    blackPlayerLayout->addWidget(m_blackReadyInLobbyCheckBox);

    auto blackTurtleIconLabel = new QLabel();
    auto blackTurtleIcon = QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Black));
    auto scaledBlackTurtleIcon = blackTurtleIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                                        Qt::KeepAspectRatio, Qt::SmoothTransformation);
    blackTurtleIconLabel->setPixmap(scaledBlackTurtleIcon);
    blackPlayerLayout->addWidget(blackTurtleIconLabel);

    std::string openString = "Open";
    bool blackPlayerJoined = !m_blackPlayerName.empty();
    bool redPlayerJoined = !m_redPlayerName.empty();

    m_blackPlayerNameLabel = new QLabel(blackPlayerJoined ? m_blackPlayerName.c_str() : openString.c_str());
    m_blackPlayerNameLabel->setEnabled(blackPlayerJoined);
    blackPlayerLayout->addWidget(m_blackPlayerNameLabel);

    if (m_playerColor == TurtlePieceColor::Black)
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
    connect(m_blackReadyInLobbyCheckBox, &QCheckBox::stateChanged, this,
            &CheckersMainMenuFrame::handleBlackReadyButtonToggled);

    inLobbyLayout->addLayout(blackPlayerLayout);

    auto redPlayerLayout = new QHBoxLayout();

    m_redReadyInLobbyCheckBox = new QCheckBox(readyInLobbyString.c_str());
    redPlayerLayout->addWidget(m_redReadyInLobbyCheckBox);

    auto redTurtleIconLabel = new QLabel();
    auto redTurtleIcon = QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Red));
    auto scaledRedTurtleIcon = redTurtleIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                                    Qt::KeepAspectRatio, Qt::SmoothTransformation);
    redTurtleIconLabel->setPixmap(scaledRedTurtleIcon);
    redPlayerLayout->addWidget(redTurtleIconLabel);

    m_redPlayerNameLabel = new QLabel(redPlayerJoined ? m_redPlayerName.c_str() : openString.c_str());
    m_redPlayerNameLabel->setEnabled(redPlayerJoined);
    redPlayerLayout->addWidget(m_redPlayerNameLabel);

    if (m_playerColor == TurtlePieceColor::Red)
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
    connect(m_redReadyInLobbyCheckBox, &QCheckBox::stateChanged, this,
            &CheckersMainMenuFrame::handleRedReadyButtonToggled);

    inLobbyLayout->addLayout(redPlayerLayout);

    auto inLobbyButtonLayout = new QHBoxLayout();

    std::string leaveLobbyInLobbyString = "Leave Lobby";
    auto leaveLobbyInLobbyButton = new QPushButton(leaveLobbyInLobbyString.c_str());
    connect(leaveLobbyInLobbyButton, &QPushButton::released, this,
            &CheckersMainMenuFrame::handleLeaveLobbyButton);
    inLobbyButtonLayout->addWidget(leaveLobbyInLobbyButton);

    inLobbyLayout->addLayout(inLobbyButtonLayout);

    return inLobbyLayoutWidget;
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

void CheckersMainMenuFrame::setPlayerName(const std::string &playerName)
{
    m_playerNameLineEdit->setText(playerName.c_str());
}

void CheckersMainMenuFrame::playerJoinedLobby(const std::string &playerName, TurtlePieceColor playerColor)
{
    if (playerName == m_playerName)
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
}

void CheckersMainMenuFrame::playerLeftLobby(const std::string &playerName)
{
    if (playerName == m_playerName)
    {
        // Do nothing
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
}

void CheckersMainMenuFrame::setPlayerReady(const std::string &playerName, bool ready)
{
    if (playerName == m_playerName)
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
}

void CheckersMainMenuFrame::validatePlayerNameText(const QString &playerName)
{
    QString playerNameCopy = playerName; // Remove the const
    int pos = 0;
    QValidator::State state = m_playerNameLineEdit->validator()->validate(playerNameCopy, pos);
    if (!playerName.isEmpty() && state == QValidator::Acceptable)
    {
        m_playerNameLineEdit->setProperty("valid", true);
        m_createLobbyButton->setEnabled(true);
        m_joinLobbyButton->setEnabled(true);
        m_playerName = playerName.toStdString();
    }
    else if (playerName.isEmpty() || state == QValidator::Invalid)
    {
        m_playerNameLineEdit->setProperty("valid", false);
        m_createLobbyButton->setEnabled(false);
        m_joinLobbyButton->setEnabled(false);
        m_playerName = "";
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

void CheckersMainMenuFrame::validatelobbyNameText(const QString &lobbyName)
{
    QString lobbyNameCopy = lobbyName; // Remove the const
    int pos = 0;
    QValidator::State state = m_lobbyNameLineEdit->validator()->validate(lobbyNameCopy, pos);
    if (!lobbyName.isEmpty() && state == QValidator::Acceptable)
    {
        m_lobbyNameLineEdit->setProperty("valid", true);
        m_commitCreateLobbyButton->setEnabled(true);
        m_lobbyName = lobbyName.toStdString();
    }
    else if (lobbyName.isEmpty() || state == QValidator::Invalid)
    {
        m_lobbyNameLineEdit->setProperty("valid", false);
        m_commitCreateLobbyButton->setEnabled(false);
        m_lobbyName = "";
    }
    else // Intermediate
    {
        // Do nothing
    }
    // Update the style
    m_lobbyNameLineEdit->style()->unpolish(m_lobbyNameLineEdit);
    m_lobbyNameLineEdit->style()->polish(m_lobbyNameLineEdit);
    m_lobbyNameLineEdit->update();
}

void CheckersMainMenuFrame::displayLobbyList(const std::vector<std::string> &lobbyNames,
                                             const std::vector<std::string> &lobbyIds,
                                             const std::vector<std::string> &blackPlayerNames,
                                             const std::vector<std::string> &redPlayerNames)
{
    m_lobbyNames = lobbyNames;
    m_lobbyIds = lobbyIds;
    m_blackPlayerNames = blackPlayerNames;
    m_redPlayerNames = redPlayerNames;
    m_windowLayout->insertWidget(JOIN_LOBBY_INDEX, createJoinLobbyScreen());
    m_windowLayout->setCurrentIndex(JOIN_LOBBY_INDEX);
}

void CheckersMainMenuFrame::handleCreateLobbyButton()
{
    if (m_createLobbyRandomRadioButton)
    {
        m_createLobbyRandomRadioButton->setChecked(true);
    }
    m_windowLayout->setCurrentIndex(CREATE_LOBBY_INDEX);
}

void CheckersMainMenuFrame::handleCancelCreateLobbyButton()
{
    m_playerDesiredColor = TurtlePieceColor::None;
    m_lobbyNameLineEdit->clear();
    m_lobbyNameLineEdit->setProperty("valid", false);
    m_lobbyNameLineEdit->style()->unpolish(m_lobbyNameLineEdit);
    m_lobbyNameLineEdit->style()->polish(m_lobbyNameLineEdit);
    m_lobbyNameLineEdit->update();
    m_commitCreateLobbyButton->setEnabled(false);
    m_playerNameLineEdit->setText(m_playerName.c_str());
    m_windowLayout->setCurrentIndex(MAIN_MENU_INDEX);
}

void CheckersMainMenuFrame::handleCommitCreateLobbyButton()
{
    m_playerWindow->createLobby(m_playerName, m_lobbyName, m_playerDesiredColor);
}

void CheckersMainMenuFrame::handleCommitJoinLobbyButton(size_t lobbyIndex)
{
    m_playerWindow->joinLobby(m_playerName, m_lobbyNames[lobbyIndex], m_lobbyIds[lobbyIndex], m_playerDesiredColor);
}

void CheckersMainMenuFrame::handleRefreshJoinLobbyButton()
{
    // After the response, we will create and move to a new (updated) version of the lobby list screen
    m_playerWindow->getLobbyList();
}

void CheckersMainMenuFrame::handleCancelJoinLobbyButton()
{
    m_playerDesiredColor = TurtlePieceColor::None;
    m_playerNameLineEdit->setText(m_playerName.c_str());
    m_windowLayout->setCurrentIndex(MAIN_MENU_INDEX);
}

void CheckersMainMenuFrame::handleJoinLobbyButton()
{
    m_playerWindow->getLobbyList(); // After the response, we will move to the lobby list screen
}

void CheckersMainMenuFrame::handleLeaveLobbyButton()
{
    m_lobbyName = "";
    m_lobbyId = "";
    m_playerDesiredColor = TurtlePieceColor::None;
    m_playerColor = TurtlePieceColor::None;
    m_blackPlayerName = "";
    m_redPlayerName = "";
    m_blackPlayerReady = false;
    m_redPlayerReady = false;
    m_playerWindow->leaveLobby();
    m_playerNameLineEdit->setText(m_playerName.c_str());
    m_windowLayout->setCurrentIndex(MAIN_MENU_INDEX);
}

void CheckersMainMenuFrame::handleBlackReadyButtonToggled(int state)
{
    m_blackPlayerReady = (state == static_cast<int>(Qt::CheckState::Checked));
    if (m_playerColor == TurtlePieceColor::Black)
    {
        m_playerWindow->setReady((state == static_cast<int>(Qt::CheckState::Checked)));
    }
}

void CheckersMainMenuFrame::handleRedReadyButtonToggled(int state)
{
    m_redPlayerReady = (state == static_cast<int>(Qt::CheckState::Checked));
    if (m_playerColor == TurtlePieceColor::Red)
    {
        m_playerWindow->setReady((state == static_cast<int>(Qt::CheckState::Checked)));
    }
}

void CheckersMainMenuFrame::handleQuitButton()
{
    m_playerWindow->close();
}

void CheckersMainMenuFrame::onBlackTurtleToggled(bool checked)
{
    if (checked)
    {
        m_playerDesiredColor = TurtlePieceColor::Black;
    }
}

void CheckersMainMenuFrame::onRandomTurtleToggled(bool checked)
{
    if (checked)
    {
        m_playerDesiredColor = TurtlePieceColor::None;
    }
}

void CheckersMainMenuFrame::onRedTurtleToggled(bool checked)
{
    if (checked)
    {
        m_playerDesiredColor = TurtlePieceColor::Red;
    }
}

void CheckersMainMenuFrame::connectedToLobby(const std::string &lobbyName,
                                             const std::string &lobbyId,
                                             const std::string &blackPlayerName,
                                             const std::string &redPlayerName,
                                             bool blackPlayerReady,
                                             bool redPlayerReady)
{
    m_lobbyName = lobbyName;
    m_lobbyId = lobbyId;
    m_blackPlayerName = blackPlayerName;
    m_redPlayerName = redPlayerName;
    m_blackPlayerReady = blackPlayerReady;
    m_redPlayerReady = redPlayerReady;
    if (m_playerName == m_blackPlayerName)
    {
        m_playerColor = TurtlePieceColor::Black;
    }
    else if (m_playerName == m_redPlayerName)
    {
        m_playerColor = TurtlePieceColor::Red;
    }
    else
    {
        m_playerColor = TurtlePieceColor::None; // Error
    }
    m_windowLayout->insertWidget(IN_LOBBY_INDEX, createInLobbyScreen());
    m_windowLayout->setCurrentIndex(IN_LOBBY_INDEX);
}