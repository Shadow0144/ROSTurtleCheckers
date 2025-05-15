#include "player/frame/CreateLobbyFrame.hpp"

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
#include "player/Parameters.hpp"
#include "player/CheckersPlayerWindow.hpp"
#include "player/ImageLibrary.hpp"

CreateLobbyFrame::CreateLobbyFrame(
    CheckersPlayerWindow *parentWindow)
    : QFrame(parentWindow, Qt::WindowFlags())
{
    m_playerWindow = parentWindow;

    m_playerDesiredColor = TurtlePieceColor::None;

    auto createLobbyLayout = new QVBoxLayout(this);
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
    connect(m_lobbyNameLineEdit, &QLineEdit::textChanged, this, &CreateLobbyFrame::validateLobbyNameText);
    createLobbyLayout->addWidget(m_lobbyNameLineEdit);

    auto lobbyPasswordLabel = new QLabel("Lobby password");
    createLobbyLayout->addWidget(lobbyPasswordLabel);

    m_createLobbyPasswordLineEdit = new QLineEdit();
    m_createLobbyPasswordLineEdit->setEchoMode(QLineEdit::Password);
    m_createLobbyPasswordLineEdit->setProperty("in_use", false);
    connect(m_createLobbyPasswordLineEdit, &QLineEdit::textChanged, this, &CreateLobbyFrame::onCreateLobbyPasswordTextChanged);
    createLobbyLayout->addWidget(m_createLobbyPasswordLineEdit);

    auto createLobbyDesiredColorLayout = new QHBoxLayout();
    createLobbyDesiredColorLayout->setAlignment(Qt::AlignCenter);

    m_createLobbyBlackRadioButton = new QRadioButton();
    m_createLobbyRandomRadioButton = new QRadioButton();
    m_createLobbyRedRadioButton = new QRadioButton();

    m_createLobbyBlackRadioButton->setIcon(QIcon(QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Black))));
    m_createLobbyRandomRadioButton->setIcon(QIcon(QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::None))));
    m_createLobbyRedRadioButton->setIcon(QIcon(QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Red))));

    m_createLobbyRandomRadioButton->setChecked(true);

    connect(m_createLobbyBlackRadioButton, &QRadioButton::toggled, this, &CreateLobbyFrame::onBlackTurtleToggled);
    connect(m_createLobbyRandomRadioButton, &QRadioButton::toggled, this, &CreateLobbyFrame::onRandomTurtleToggled);
    connect(m_createLobbyRedRadioButton, &QRadioButton::toggled, this, &CreateLobbyFrame::onRedTurtleToggled);

    createLobbyDesiredColorLayout->addWidget(m_createLobbyBlackRadioButton);
    createLobbyDesiredColorLayout->addWidget(m_createLobbyRandomRadioButton);
    createLobbyDesiredColorLayout->addWidget(m_createLobbyRedRadioButton);

    createLobbyLayout->addLayout(createLobbyDesiredColorLayout);

    auto createLobbyButtonLayout = new QHBoxLayout();

    std::string commitCreateLobbyString = "Create Lobby";
    m_createLobbyButton = new QPushButton(commitCreateLobbyString.c_str());
    m_createLobbyButton->setEnabled(false);
    connect(m_createLobbyButton, &QPushButton::released, this,
            &CreateLobbyFrame::handleCreateLobbyButton);
    createLobbyButtonLayout->addWidget(m_createLobbyButton);

    std::string cancelCreateLobbyString = "Cancel";
    auto cancelCreateLobbyButton = new QPushButton(cancelCreateLobbyString.c_str());
    connect(cancelCreateLobbyButton, &QPushButton::released, this,
            &CreateLobbyFrame::handleCancelButton);
    createLobbyButtonLayout->addWidget(cancelCreateLobbyButton);

    createLobbyLayout->addLayout(createLobbyButtonLayout);
}

CreateLobbyFrame::~CreateLobbyFrame()
{
}

void CreateLobbyFrame::showEvent(QShowEvent *event)
{
	(void)event; // NO LINT
    
    m_lobbyNameLineEdit->clear();
    m_lobbyNameLineEdit->setProperty("valid", false);
    m_lobbyNameLineEdit->style()->unpolish(m_lobbyNameLineEdit);
    m_lobbyNameLineEdit->style()->polish(m_lobbyNameLineEdit);
    m_lobbyNameLineEdit->update();
    m_createLobbyPasswordLineEdit->clear();
    m_createLobbyPasswordLineEdit->setProperty("in_use", false);
    m_createLobbyPasswordLineEdit->style()->unpolish(m_createLobbyPasswordLineEdit);
    m_createLobbyPasswordLineEdit->style()->polish(m_createLobbyPasswordLineEdit);
    m_createLobbyPasswordLineEdit->update();
    m_createLobbyButton->setEnabled(false);
    m_createLobbyRandomRadioButton->setChecked(true);
}

void CreateLobbyFrame::validateLobbyNameText(const QString &lobbyName)
{
    QString lobbyNameCopy = lobbyName; // Remove the const
    int pos = 0;
    QValidator::State state = m_lobbyNameLineEdit->validator()->validate(lobbyNameCopy, pos);
    if (!lobbyName.isEmpty() && state == QValidator::Acceptable)
    {
        m_lobbyNameLineEdit->setProperty("valid", true);
        m_createLobbyButton->setEnabled(true);
        Parameters::setLobbyName(lobbyName.toStdString());
    }
    else if (lobbyName.isEmpty() || state == QValidator::Invalid)
    {
        m_lobbyNameLineEdit->setProperty("valid", false);
        m_createLobbyButton->setEnabled(false);
        Parameters::setLobbyName("");
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

void CreateLobbyFrame::onCreateLobbyPasswordTextChanged(const QString &lobbyPassword)
{
    m_createLobbyPasswordLineEdit->setProperty("in_use", !lobbyPassword.isEmpty());
    // Update the style
    m_createLobbyPasswordLineEdit->style()->unpolish(m_createLobbyPasswordLineEdit);
    m_createLobbyPasswordLineEdit->style()->polish(m_createLobbyPasswordLineEdit);
    m_createLobbyPasswordLineEdit->update();
}

void CreateLobbyFrame::handleCancelButton()
{
    m_playerWindow->moveToMainMenuFrame();
}

void CreateLobbyFrame::handleCreateLobbyButton()
{
    auto lobbyPassword = m_createLobbyPasswordLineEdit->text().toStdString();
    m_playerWindow->createLobby(lobbyPassword);
}

void CreateLobbyFrame::onBlackTurtleToggled(bool checked)
{
    if (checked)
    {
        m_playerDesiredColor = TurtlePieceColor::Black;
        Parameters::setPlayerColor(m_playerDesiredColor);
    }
}

void CreateLobbyFrame::onRandomTurtleToggled(bool checked)
{
    if (checked)
    {
        m_playerDesiredColor = TurtlePieceColor::None;
        Parameters::setPlayerColor(m_playerDesiredColor);
    }
}

void CreateLobbyFrame::onRedTurtleToggled(bool checked)
{
    if (checked)
    {
        m_playerDesiredColor = TurtlePieceColor::Red;
        Parameters::setPlayerColor(m_playerDesiredColor);
    }
}