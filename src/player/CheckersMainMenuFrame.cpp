#include "player/CheckersMainMenuFrame.hpp"

#include <QFrame>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPointF>
#include <QStackedLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
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

    // Create the style sheets
    std::string backgroundColorStyleString = "background-color: rgb(" +
                                             std::to_string(BG_RGB[0]) + ", " +
                                             std::to_string(BG_RGB[1]) + ", " +
                                             std::to_string(BG_RGB[2]) + ");";
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
    std::string selectedRadioButtonStyleString = textColorStyleString + "border: 1px solid aqua;";
    std::string unselectedRadioButtonStyleString = textColorStyleString + "border: 1px solid black;";
    std::string defaultButtonStyleString = textColorStyleString + "border: 1px solid white;";
    std::string disabledButtonStyleString = textDisabledColorStyleString + "border: 1px solid dimGray;";
    std::string readyButtonStyleString = textColorStyleString + "border: 1px solid green;";
    setStyleSheet(QString(backgroundColorStyleString.c_str()));
    m_labelStyleSheet = textColorStyleString.c_str();
    m_openNameLabelStyleSheet = textDisabledColorStyleString.c_str();
    m_lineEditValidStyleSheet = textValidColorStyleString.c_str();
    m_lineEditInvalidStyleSheet = textInvalidColorStyleString.c_str();
    m_buttonDefaultStyleSheet = defaultButtonStyleString.c_str();
    m_buttonDisabledStyleSheet = disabledButtonStyleString.c_str();
    m_selectedRadioButtonStyleSheet = selectedRadioButtonStyleString.c_str();
    m_unselectedRadioButtonStyleSheet = unselectedRadioButtonStyleString.c_str();
    m_unreadyButtonStyleSheet = defaultButtonStyleString.c_str();
    m_disabledReadyButtonStyleSheet = disabledButtonStyleString.c_str();
    m_readyButtonStyleSheet = readyButtonStyleString.c_str();

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
    titleLabel->setStyleSheet(m_labelStyleSheet);
    mainLayout->addWidget(titleLabel);

    auto playerNameLabel = new QLabel("Player name");
    playerNameLabel->setStyleSheet(m_labelStyleSheet);
    mainLayout->addWidget(playerNameLabel);

    m_playerNameLineEdit = new QLineEdit();
    auto playerNameValidator = new QRegularExpressionValidator(
        QRegularExpression("[a-zA-Z0-9_]+"));
    m_playerNameLineEdit->setValidator(playerNameValidator);
    m_playerNameLineEdit->setStyleSheet(m_lineEditInvalidStyleSheet);
    connect(m_playerNameLineEdit, &QLineEdit::textChanged, this, &CheckersMainMenuFrame::validatePlayerNameText);
    mainLayout->addWidget(m_playerNameLineEdit);

    auto buttonLayout = new QHBoxLayout();

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
    auto quitButton = new QPushButton(quitString.c_str());
    quitButton->setStyleSheet(m_buttonDefaultStyleSheet);
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
    createLobbyTitleLabel->setStyleSheet(m_labelStyleSheet);
    createLobbyLayout->addWidget(createLobbyTitleLabel);

    auto lobbyNameLabel = new QLabel("Lobby name");
    lobbyNameLabel->setStyleSheet(m_labelStyleSheet);
    createLobbyLayout->addWidget(lobbyNameLabel);

    m_lobbyNameLineEdit = new QLineEdit();
    auto lobbyNameValidator = new QRegularExpressionValidator(
        QRegularExpression("[a-zA-Z0-9_]+"));
    m_lobbyNameLineEdit->setValidator(lobbyNameValidator);
    m_lobbyNameLineEdit->setStyleSheet(m_lineEditInvalidStyleSheet);
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

    m_createLobbyBlackRadioButton->setStyleSheet(m_unselectedRadioButtonStyleSheet);
    m_createLobbyRandomRadioButton->setStyleSheet(m_unselectedRadioButtonStyleSheet);
    m_createLobbyRedRadioButton->setStyleSheet(m_unselectedRadioButtonStyleSheet);

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
    m_commitCreateLobbyButton->setStyleSheet(m_buttonDisabledStyleSheet);
    connect(m_commitCreateLobbyButton, &QPushButton::released, this,
            &CheckersMainMenuFrame::handleCommitCreateLobbyButton);
    createLobbyButtonLayout->addWidget(m_commitCreateLobbyButton);

    std::string cancelCreateLobbyString = "Cancel";
    auto cancelCreateLobbyButton = new QPushButton(cancelCreateLobbyString.c_str());
    cancelCreateLobbyButton->setStyleSheet(m_buttonDefaultStyleSheet);
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
    joinLobbyTitleLabel->setStyleSheet(m_labelStyleSheet);
    joinLobbyLayout->addWidget(joinLobbyTitleLabel);

    auto lobbiesLabel = new QLabel("Lobbies:");
    joinLobbyTitleLabel->setStyleSheet(m_labelStyleSheet);
    joinLobbyLayout->addWidget(lobbiesLabel);

    auto lobbyListFrame = new QFrame();
    lobbyListFrame->setFrameStyle(QFrame::Box | QFrame::Plain);
    lobbyListFrame->setStyleSheet("QFrame { border: 1px solid aqua; } QLabel { border: 0px; }");

    auto lobbyListLayout = new QVBoxLayout(lobbyListFrame);

    const auto numLobbies = m_lobbyNames.size();
    if (m_lobbyIds.size() != numLobbies)
    {
        std::cerr << "Lobby name vector does not match size of lobby id vector" << std::endl;
        return nullptr;
    }
    for (size_t i = 0u; i < numLobbies; i++)
    {
        auto lobbyLayout = new QHBoxLayout();

        auto lobbyNameLayout = new QHBoxLayout();

        auto lobbyNameLabel = new QLabel(m_lobbyNames[i].c_str());
        lobbyNameLabel->setStyleSheet(m_labelStyleSheet);
        lobbyNameLayout->addWidget(lobbyNameLabel);

        std::string lobbyIdWithHash = "#" + m_lobbyIds[i];
        auto lobbyIdLabel = new QLabel(lobbyIdWithHash.c_str());
        lobbyIdLabel->setStyleSheet(m_openNameLabelStyleSheet);
        lobbyNameLayout->addWidget(lobbyIdLabel);

        lobbyLayout->addLayout(lobbyNameLayout);

        auto blackTurtleIconLabel = new QLabel();
        blackTurtleIconLabel->setPixmap(QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Black)));
        lobbyLayout->addWidget(blackTurtleIconLabel);

        std::string openString = "Open";
        bool blackPlayerJoined = !m_blackPlayerNames[i].empty();
        bool redPlayerJoined = !m_redPlayerNames[i].empty();

        auto blackPlayerNameLabel = new QLabel(blackPlayerJoined ? m_blackPlayerNames[i].c_str() : openString.c_str());
        blackPlayerNameLabel->setStyleSheet(blackPlayerJoined ? m_labelStyleSheet : m_openNameLabelStyleSheet);
        lobbyLayout->addWidget(blackPlayerNameLabel);

        auto redTurtleIconLabel = new QLabel();
        redTurtleIconLabel->setPixmap(QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Red)));
        lobbyLayout->addWidget(redTurtleIconLabel);

        auto redPlayerNameLabel = new QLabel(redPlayerJoined ? m_redPlayerNames[i].c_str() : openString.c_str());
        redPlayerNameLabel->setStyleSheet(redPlayerJoined ? m_labelStyleSheet : m_openNameLabelStyleSheet);
        lobbyLayout->addWidget(redPlayerNameLabel);

        std::string joinLobbyString = "Join";
        auto joinLobbyButton = new QPushButton(joinLobbyString.c_str());
        if (!blackPlayerJoined || !redPlayerJoined)
        {
            joinLobbyButton->setEnabled(true);
            joinLobbyButton->setStyleSheet(m_buttonDefaultStyleSheet);
        }
        else
        {
            // Lobby is full
            joinLobbyButton->setEnabled(false);
            joinLobbyButton->setStyleSheet(m_buttonDisabledStyleSheet);
        }
        connect(joinLobbyButton, &QPushButton::released, this,
                [i, this]()
                { this->handleCommitJoinLobbyButton(i); });
        lobbyLayout->addWidget(joinLobbyButton);

        lobbyListLayout->addLayout(lobbyLayout);
    }

    joinLobbyLayout->addWidget(lobbyListFrame);

    auto joinLobbyDesiredColorLayout = new QHBoxLayout();
    joinLobbyDesiredColorLayout->setAlignment(Qt::AlignCenter);

    m_joinLobbyBlackRadioButton = new QRadioButton();
    m_joinLobbyRandomRadioButton = new QRadioButton();
    m_joinLobbyRedRadioButton = new QRadioButton();

    m_joinLobbyBlackRadioButton->setIcon(QIcon(QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Black))));
    m_joinLobbyRandomRadioButton->setIcon(QIcon(QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::None))));
    m_joinLobbyRedRadioButton->setIcon(QIcon(QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Red))));

    m_joinLobbyBlackRadioButton->setStyleSheet(m_unselectedRadioButtonStyleSheet);
    m_joinLobbyRandomRadioButton->setStyleSheet(m_unselectedRadioButtonStyleSheet);
    m_joinLobbyRedRadioButton->setStyleSheet(m_unselectedRadioButtonStyleSheet);

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
    refreshJoinLobbyButton->setStyleSheet(m_buttonDefaultStyleSheet);
    connect(refreshJoinLobbyButton, &QPushButton::released, this,
            &CheckersMainMenuFrame::handleRefreshJoinLobbyButton);
    joinLobbyButtonLayout->addWidget(refreshJoinLobbyButton);

    std::string canceljoinLobbyString = "Cancel";
    auto cancelJoinLobbyButton = new QPushButton(canceljoinLobbyString.c_str());
    cancelJoinLobbyButton->setStyleSheet(m_buttonDefaultStyleSheet);
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
    inLobbyTitleLabel->setStyleSheet(m_labelStyleSheet);
    inLobbyLayout->addWidget(inLobbyTitleLabel);

    auto lobbyNameLayout = new QHBoxLayout();

    auto lobbyNameLabel = new QLabel(m_lobbyName.c_str());
    lobbyNameLabel->setStyleSheet(m_labelStyleSheet);
    lobbyNameLayout->addWidget(lobbyNameLabel);

    std::string lobbyIdWithHash = "#" + m_lobbyId;
    auto lobbyIdLabel = new QLabel(lobbyIdWithHash.c_str());
    lobbyIdLabel->setStyleSheet(m_openNameLabelStyleSheet);
    lobbyNameLayout->addWidget(lobbyIdLabel);

    inLobbyLayout->addLayout(lobbyNameLayout);

    std::string readyInLobbyString = "Ready";

    auto blackPlayerLayout = new QHBoxLayout();

    m_blackReadyInLobbyCheckBox = new QCheckBox(readyInLobbyString.c_str());
    blackPlayerLayout->addWidget(m_blackReadyInLobbyCheckBox);

    auto blackTurtleIconLabel = new QLabel();
    blackTurtleIconLabel->setPixmap(QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Black)));
    blackPlayerLayout->addWidget(blackTurtleIconLabel);

    m_blackPlayerNameLabel = new QLabel(m_blackPlayerName.c_str());
    blackPlayerLayout->addWidget(m_blackPlayerNameLabel);

    if (m_playerColor == TurtlePieceColor::Black)
    {
        m_blackPlayerNameLabel->setStyleSheet(m_labelStyleSheet);
        m_blackReadyInLobbyCheckBox->setEnabled(true);
        if (m_blackPlayerReady)
        {
            m_blackReadyInLobbyCheckBox->setCheckState(Qt::Checked);
            m_blackReadyInLobbyCheckBox->setStyleSheet(m_readyButtonStyleSheet);
        }
        else
        {
            m_blackReadyInLobbyCheckBox->setCheckState(Qt::Unchecked);
            m_blackReadyInLobbyCheckBox->setStyleSheet(m_unreadyButtonStyleSheet);
        }
    }
    else
    {
        m_blackPlayerNameLabel->setStyleSheet(m_openNameLabelStyleSheet);
        m_blackReadyInLobbyCheckBox->setEnabled(false);
        if (m_blackPlayerReady)
        {
            m_blackReadyInLobbyCheckBox->setCheckState(Qt::Checked);
            m_blackReadyInLobbyCheckBox->setStyleSheet(m_readyButtonStyleSheet);
        }
        else
        {
            m_blackReadyInLobbyCheckBox->setCheckState(Qt::Unchecked);
            m_blackReadyInLobbyCheckBox->setStyleSheet(m_disabledReadyButtonStyleSheet);
        }
    }
    connect(m_blackReadyInLobbyCheckBox, &QCheckBox::stateChanged, this,
            &CheckersMainMenuFrame::handleBlackReadyButtonToggled);

    inLobbyLayout->addLayout(blackPlayerLayout);

    auto redPlayerLayout = new QHBoxLayout();

    m_redReadyInLobbyCheckBox = new QCheckBox(readyInLobbyString.c_str());
    redPlayerLayout->addWidget(m_redReadyInLobbyCheckBox);

    auto redTurtleIconLabel = new QLabel();
    redTurtleIconLabel->setPixmap(QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Red)));
    redPlayerLayout->addWidget(redTurtleIconLabel);

    m_redPlayerNameLabel = new QLabel(m_redPlayerName.c_str());
    redPlayerLayout->addWidget(m_redPlayerNameLabel);

    if (m_playerColor == TurtlePieceColor::Red)
    {
        m_redPlayerNameLabel->setStyleSheet(m_labelStyleSheet);
        m_redReadyInLobbyCheckBox->setEnabled(true);
        if (m_redPlayerReady)
        {
            m_redReadyInLobbyCheckBox->setCheckState(Qt::Checked);
            m_redReadyInLobbyCheckBox->setStyleSheet(m_readyButtonStyleSheet);
        }
        else
        {
            m_redReadyInLobbyCheckBox->setCheckState(Qt::Unchecked);
            m_redReadyInLobbyCheckBox->setStyleSheet(m_unreadyButtonStyleSheet);
        }
    }
    else
    {
        m_redPlayerNameLabel->setStyleSheet(m_openNameLabelStyleSheet);
        m_redReadyInLobbyCheckBox->setEnabled(false);
        if (m_redPlayerReady)
        {
            m_redReadyInLobbyCheckBox->setCheckState(Qt::Checked);
            m_redReadyInLobbyCheckBox->setStyleSheet(m_readyButtonStyleSheet);
        }
        else
        {
            m_redReadyInLobbyCheckBox->setCheckState(Qt::Unchecked);
            m_redReadyInLobbyCheckBox->setStyleSheet(m_disabledReadyButtonStyleSheet);
        }
    }
    connect(m_redReadyInLobbyCheckBox, &QCheckBox::stateChanged, this,
            &CheckersMainMenuFrame::handleRedReadyButtonToggled);

    inLobbyLayout->addLayout(redPlayerLayout);

    auto inLobbyButtonLayout = new QHBoxLayout();

    std::string leaveLobbyInLobbyString = "Leave Lobby";
    auto leaveLobbyInLobbyButton = new QPushButton(leaveLobbyInLobbyString.c_str());
    leaveLobbyInLobbyButton->setStyleSheet(m_buttonDefaultStyleSheet);
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
    }
    break;
    case TurtlePieceColor::Red:
    {
        m_redPlayerName = playerName;
        m_redPlayerNameLabel->setText(m_redPlayerName.c_str());
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
        m_blackPlayerNameLabel->clear();
        m_blackReadyInLobbyCheckBox->setCheckState(Qt::Unchecked);
        m_blackPlayerName.clear();
    }
    else if (playerName == m_redPlayerName)
    {
        m_redPlayerNameLabel->clear();
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
        m_playerNameLineEdit->setStyleSheet(m_lineEditValidStyleSheet);
        m_createLobbyButton->setEnabled(true);
        m_createLobbyButton->setStyleSheet(m_buttonDefaultStyleSheet);
        m_joinLobbyButton->setEnabled(true);
        m_joinLobbyButton->setStyleSheet(m_buttonDefaultStyleSheet);
        m_playerName = playerName.toStdString();
    }
    else if (playerName.isEmpty() || state == QValidator::Invalid)
    {
        m_playerNameLineEdit->setStyleSheet(m_lineEditInvalidStyleSheet);
        m_createLobbyButton->setEnabled(false);
        m_createLobbyButton->setStyleSheet(m_buttonDisabledStyleSheet);
        m_joinLobbyButton->setEnabled(false);
        m_joinLobbyButton->setStyleSheet(m_buttonDisabledStyleSheet);
        m_playerName = "";
    }
    else // Intermediate
    {
        // Do nothing
    }
}

void CheckersMainMenuFrame::validatelobbyNameText(const QString &lobbyName)
{
    QString lobbyNameCopy = lobbyName; // Remove the const
    int pos = 0;
    QValidator::State state = m_lobbyNameLineEdit->validator()->validate(lobbyNameCopy, pos);
    if (!lobbyName.isEmpty() && state == QValidator::Acceptable)
    {
        m_lobbyNameLineEdit->setStyleSheet(m_lineEditValidStyleSheet);
        m_commitCreateLobbyButton->setEnabled(true);
        m_commitCreateLobbyButton->setStyleSheet(m_buttonDefaultStyleSheet);
        m_lobbyName = lobbyName.toStdString();
    }
    else if (lobbyName.isEmpty() || state == QValidator::Invalid)
    {
        m_lobbyNameLineEdit->setStyleSheet(m_lineEditInvalidStyleSheet);
        m_commitCreateLobbyButton->setEnabled(false);
        m_commitCreateLobbyButton->setStyleSheet(m_buttonDisabledStyleSheet);
        m_lobbyName = "";
    }
    else // Intermediate
    {
        // Do nothing
    }
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
    m_lobbyNameLineEdit->setStyleSheet(m_lineEditInvalidStyleSheet);
    m_commitCreateLobbyButton->setEnabled(false);
    m_commitCreateLobbyButton->setStyleSheet(m_buttonDisabledStyleSheet);
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
    if (m_blackPlayerReady)
    {
        m_blackReadyInLobbyCheckBox->setStyleSheet(m_readyButtonStyleSheet);
    }
    else
    {
        if (m_playerColor == TurtlePieceColor::Black)
        {
            m_blackReadyInLobbyCheckBox->setStyleSheet(m_unreadyButtonStyleSheet);
        }
        else
        {
            m_blackReadyInLobbyCheckBox->setStyleSheet(m_disabledReadyButtonStyleSheet);
        }
    }
}

void CheckersMainMenuFrame::handleRedReadyButtonToggled(int state)
{
    m_redPlayerReady = (state == static_cast<int>(Qt::CheckState::Checked));
    if (m_playerColor == TurtlePieceColor::Red)
    {
        m_playerWindow->setReady((state == static_cast<int>(Qt::CheckState::Checked)));
    }
    if (m_redPlayerReady)
    {
        m_redReadyInLobbyCheckBox->setStyleSheet(m_readyButtonStyleSheet);
    }
    else
    {
        if (m_playerColor == TurtlePieceColor::Red)
        {
            m_redReadyInLobbyCheckBox->setStyleSheet(m_unreadyButtonStyleSheet);
        }
        else
        {
            m_redReadyInLobbyCheckBox->setStyleSheet(m_disabledReadyButtonStyleSheet);
        }
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
        if (m_createLobbyBlackRadioButton)
        {
            m_createLobbyBlackRadioButton->setStyleSheet(m_selectedRadioButtonStyleSheet);
        }
        if (m_joinLobbyBlackRadioButton)
        {
            m_joinLobbyBlackRadioButton->setStyleSheet(m_selectedRadioButtonStyleSheet);
        }
    }
    else
    {
        if (m_createLobbyBlackRadioButton)
        {
            m_createLobbyBlackRadioButton->setStyleSheet(m_unselectedRadioButtonStyleSheet);
        }
        if (m_joinLobbyBlackRadioButton)
        {
            m_joinLobbyBlackRadioButton->setStyleSheet(m_unselectedRadioButtonStyleSheet);
        }
    }
}

void CheckersMainMenuFrame::onRandomTurtleToggled(bool checked)
{
    if (checked)
    {
        m_playerDesiredColor = TurtlePieceColor::None;
        if (m_createLobbyRandomRadioButton)
        {
            m_createLobbyRandomRadioButton->setStyleSheet(m_selectedRadioButtonStyleSheet);
        }
        if (m_joinLobbyRandomRadioButton)
        {
            m_joinLobbyRandomRadioButton->setStyleSheet(m_selectedRadioButtonStyleSheet);
        }
    }
    else
    {
        if (m_createLobbyRandomRadioButton)
        {
            m_createLobbyRandomRadioButton->setStyleSheet(m_unselectedRadioButtonStyleSheet);
        }
        if (m_joinLobbyRandomRadioButton)
        {
            m_joinLobbyRandomRadioButton->setStyleSheet(m_unselectedRadioButtonStyleSheet);
        }
    }
}

void CheckersMainMenuFrame::onRedTurtleToggled(bool checked)
{
    if (checked)
    {
        m_playerDesiredColor = TurtlePieceColor::Red;
        if (m_createLobbyRedRadioButton)
        {
            m_createLobbyRedRadioButton->setStyleSheet(m_selectedRadioButtonStyleSheet);
        }
        if (m_joinLobbyRedRadioButton)
        {
            m_joinLobbyRedRadioButton->setStyleSheet(m_selectedRadioButtonStyleSheet);
        }
    }
    else
    {
        if (m_createLobbyRedRadioButton)
        {
            m_createLobbyRedRadioButton->setStyleSheet(m_unselectedRadioButtonStyleSheet);
        }
        if (m_joinLobbyRedRadioButton)
        {
            m_joinLobbyRedRadioButton->setStyleSheet(m_unselectedRadioButtonStyleSheet);
        }
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