#include "player/frame/CreateLobbyFrame.hpp"

#include <QFrame>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QString>
#include <QRegularExpression>
#include <QRegularExpressionValidator>
#include <QPushButton>
#include <QRadioButton>
#include <QIcon>
#include <QPixmap>

#include <cstdlib>
#include <memory>
#include <string>

#include "player/CheckersPlayerWindow.hpp"
#include "shared/CheckersConsts.hpp"
#include "player/Parameters.hpp"
#include "player/ImageLibrary.hpp"
#include "player/TitleWidget.hpp"
#include "player/LanguageSelectorWidget.hpp"

CreateLobbyFrame::CreateLobbyFrame(
    CheckersPlayerWindow *parentWindow)
    : QFrame(parentWindow, Qt::WindowFlags())
{
    m_playerWindow = parentWindow;

    m_playerDesiredColor = TurtlePieceColor::None;

    auto createLobbyLayout = new QVBoxLayout(this);
    createLobbyLayout->setAlignment(Qt::AlignCenter);

    m_languageSelector = new LanguageSelectorWidget(this);

    m_titleWidget = new TitleWidget();
    createLobbyLayout->addWidget(m_titleWidget);

    m_lobbyNameLabel = new TranslatedQLabel("Lobby name");
    createLobbyLayout->addWidget(m_lobbyNameLabel);

    m_lobbyNameLineEdit = new QLineEdit();
    m_lobbyNameLineEdit->setFixedWidth(MENU_LINE_EDIT_WIDTH);
    std::string lobbyNameRegex = "^[a-zA-Z][a-zA-Z0-9_]{0," + std::to_string(MAX_CHARS_LOBBY_NAME) + "}$";
    auto lobbyNameValidator = new QRegularExpressionValidator(QRegularExpression(lobbyNameRegex.c_str()));
    m_lobbyNameLineEdit->setValidator(lobbyNameValidator);
    m_lobbyNameLineEdit->setProperty("valid", false);
    connect(m_lobbyNameLineEdit, &QLineEdit::textChanged, this, &CreateLobbyFrame::validateLobbyNameText);
    createLobbyLayout->addWidget(m_lobbyNameLineEdit);

    m_lobbyPasswordLabel = new TranslatedQLabel("Lobby password");
    createLobbyLayout->addWidget(m_lobbyPasswordLabel);

    m_lobbyPasswordLineEdit = new QLineEdit();
    m_lobbyPasswordLineEdit->setFixedWidth(MENU_LINE_EDIT_WIDTH);
    std::string lobbyPasswordRegex = "^[a-zA-Z0-9a-zA-Z0-9\\`\\~\\!\\@\\#\\$\\%\\^\\&\\*\\(\\)\\-\\_\\=\\+\\[\\]\\{\\}\\|\\;\\:\\,\\.\\<\\>\\?]{0," + std::to_string(MAX_CHARS_LOBBY_PASS) + "}$";
    auto lobbyPasswordValidator = new QRegularExpressionValidator(QRegularExpression(lobbyPasswordRegex.c_str()));
    m_lobbyPasswordLineEdit->setValidator(lobbyPasswordValidator);
    m_lobbyPasswordLineEdit->setEchoMode(QLineEdit::Password);
    m_lobbyPasswordLineEdit->setProperty("in_use", false);
    connect(m_lobbyPasswordLineEdit, &QLineEdit::textChanged, this, &CreateLobbyFrame::validatePasswordText);
    createLobbyLayout->addWidget(m_lobbyPasswordLineEdit);

    auto createLobbyDesiredColorLayout = new QHBoxLayout();
    createLobbyDesiredColorLayout->setAlignment(Qt::AlignCenter);

    m_blackRadioButton = new QRadioButton();
    m_randomRadioButton = new QRadioButton();
    m_redRadioButton = new QRadioButton();

    m_blackRadioButton->setIcon(QIcon(QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Black))));
    m_randomRadioButton->setIcon(QIcon(QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::None))));
    m_redRadioButton->setIcon(QIcon(QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Red))));

    m_randomRadioButton->setChecked(true);

    connect(m_blackRadioButton, &QRadioButton::toggled, this, &CreateLobbyFrame::onBlackTurtleToggled);
    connect(m_randomRadioButton, &QRadioButton::toggled, this, &CreateLobbyFrame::onRandomTurtleToggled);
    connect(m_redRadioButton, &QRadioButton::toggled, this, &CreateLobbyFrame::onRedTurtleToggled);

    createLobbyDesiredColorLayout->addWidget(m_blackRadioButton);
    createLobbyDesiredColorLayout->addWidget(m_randomRadioButton);
    createLobbyDesiredColorLayout->addWidget(m_redRadioButton);

    createLobbyLayout->addLayout(createLobbyDesiredColorLayout);

    auto createLobbyButtonLayout = new QHBoxLayout();
    createLobbyButtonLayout->setAlignment(Qt::AlignCenter);

    m_createLobbyButton = new TranslatedQPushButton("Create Lobby");
    m_createLobbyButton->setFixedWidth(MENU_BUTTON_WIDTH);
    m_createLobbyButton->setEnabled(false);
    connect(m_createLobbyButton, &QPushButton::released, this,
            &CreateLobbyFrame::handleCreateLobbyButton);
    m_createLobbyButton->setDefault(true);
    connect(m_lobbyNameLineEdit, &QLineEdit::returnPressed, m_createLobbyButton, &QPushButton::click);
    connect(m_lobbyPasswordLineEdit, &QLineEdit::returnPressed, m_createLobbyButton, &QPushButton::click);
    createLobbyButtonLayout->addWidget(m_createLobbyButton);

    m_cancelCreateLobbyButton = new TranslatedQPushButton("Cancel");
    m_cancelCreateLobbyButton->setFixedWidth(MENU_BUTTON_WIDTH);
    connect(m_cancelCreateLobbyButton, &QPushButton::released, this,
            &CreateLobbyFrame::handleCancelButton);
    createLobbyButtonLayout->addWidget(m_cancelCreateLobbyButton);

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
    m_lobbyPasswordLineEdit->clear();
    m_lobbyPasswordLineEdit->setProperty("in_use", false);
    m_lobbyPasswordLineEdit->style()->unpolish(m_lobbyPasswordLineEdit);
    m_lobbyPasswordLineEdit->style()->polish(m_lobbyPasswordLineEdit);
    m_lobbyPasswordLineEdit->update();
    m_createLobbyButton->setEnabled(false);
    m_randomRadioButton->setChecked(true);

    m_lobbyNameLineEdit->setFocus();

    m_languageSelector->setCurrentIndex(static_cast<int>(Parameters::getLanguage()));
    reloadStrings();
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

void CreateLobbyFrame::validatePasswordText(const QString &lobbyPassword)
{
    m_lobbyPasswordLineEdit->setProperty("in_use", !lobbyPassword.isEmpty());
    // Update the style
    m_lobbyPasswordLineEdit->style()->unpolish(m_lobbyPasswordLineEdit);
    m_lobbyPasswordLineEdit->style()->polish(m_lobbyPasswordLineEdit);
    m_lobbyPasswordLineEdit->update();
}

void CreateLobbyFrame::handleCancelButton()
{
    m_playerWindow->moveToMainMenuFrame();
}

void CreateLobbyFrame::handleCreateLobbyButton()
{
    auto lobbyPassword = m_lobbyPasswordLineEdit->text().toStdString();
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

void CreateLobbyFrame::reloadStrings()
{
    m_titleWidget->reloadStrings();

    m_lobbyNameLabel->reloadStrings();
    m_lobbyPasswordLabel->reloadStrings();

    m_createLobbyButton->reloadStrings();
    m_cancelCreateLobbyButton->reloadStrings();
}