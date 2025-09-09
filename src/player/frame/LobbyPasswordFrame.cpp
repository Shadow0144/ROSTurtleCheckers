#include "player/frame/LobbyPasswordFrame.hpp"

#include <QFrame>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QString>
#include <QRegularExpression>
#include <QRegularExpressionValidator>
#include <QPushButton>

#include <cstdlib>
#include <memory>
#include <string>

#include "player/CheckersPlayerWindow.hpp"
#include "shared/CheckersConsts.hpp"
#include "player/Parameters.hpp"
#include "player/ImageLibrary.hpp"
#include "player/StringLibrary.hpp"
#include "player/TitleWidget.hpp"
#include "player/LanguageSelectorWidget.hpp"

LobbyPasswordFrame::LobbyPasswordFrame(
    CheckersPlayerWindow *parentWindow)
    : QFrame(parentWindow, Qt::WindowFlags())
{
    m_playerWindow = parentWindow;

    auto enterLobbyPasswordLayout = new QVBoxLayout(this);
    enterLobbyPasswordLayout->setAlignment(Qt::AlignCenter);

    m_titleWidget = new TitleWidget();
    enterLobbyPasswordLayout->addWidget(m_titleWidget);

    m_languageSelector = new LanguageSelectorWidget(this);

    auto lobbyNameLayout = new QHBoxLayout();

    m_lobbyNameLabel = new QLabel("");
    lobbyNameLayout->addWidget(m_lobbyNameLabel);

    m_lobbyIdLabel = new QLabel("");
    lobbyNameLayout->addWidget(m_lobbyIdLabel);

    enterLobbyPasswordLayout->addLayout(lobbyNameLayout);

    m_lobbyPasswordLabel = new QLabel(StringLibrary::getTranslatedString("Lobby password"));
    enterLobbyPasswordLayout->addWidget(m_lobbyPasswordLabel);

    m_lobbyPasswordLineEdit = new QLineEdit();
    m_lobbyPasswordLineEdit->setFixedWidth(MENU_LINE_EDIT_WIDTH);
    std::string lobbyPasswordRegex = "^[a-zA-Z0-9a-zA-Z0-9\\`\\~\\!\\@\\#\\$\\%\\^\\&\\*\\(\\)\\-\\_\\=\\+\\[\\]\\{\\}\\|\\;\\:\\,\\.\\<\\>\\?]{0," + std::to_string(MAX_CHARS_LOBBY_PASS) + "}$";
    auto lobbyPasswordValidator = new QRegularExpressionValidator(QRegularExpression(lobbyPasswordRegex.c_str()));
    m_lobbyPasswordLineEdit->setValidator(lobbyPasswordValidator);
    m_lobbyPasswordLineEdit->setEchoMode(QLineEdit::Password);
    m_lobbyPasswordLineEdit->setProperty("in_use", false);
    connect(m_lobbyPasswordLineEdit, &QLineEdit::textChanged, this, &LobbyPasswordFrame::validatePasswordText);
    enterLobbyPasswordLayout->addWidget(m_lobbyPasswordLineEdit);

    m_passwordIncorrectLabel = new QLabel(StringLibrary::getTranslatedString("Incorrect password"));
    m_passwordIncorrectLabel->setProperty("error", true);
    auto passwordIncorrectLabelSizePolicy = m_passwordIncorrectLabel->sizePolicy();
    passwordIncorrectLabelSizePolicy.setRetainSizeWhenHidden(true);
    m_passwordIncorrectLabel->setSizePolicy(passwordIncorrectLabelSizePolicy);
    m_passwordIncorrectLabel->setVisible(false);
    enterLobbyPasswordLayout->addWidget(m_passwordIncorrectLabel);

    auto enterLobbyPasswordButtonLayout = new QHBoxLayout();
    enterLobbyPasswordButtonLayout->setAlignment(Qt::AlignCenter);

    m_confirmPasswordButton = new QPushButton(StringLibrary::getTranslatedString("Join Lobby"));
    m_confirmPasswordButton->setFixedWidth(MENU_BUTTON_WIDTH);
    m_confirmPasswordButton->setEnabled(false);
    connect(m_confirmPasswordButton, &QPushButton::released, this,
            &LobbyPasswordFrame::handleConfirmPasswordButton);
    connect(m_lobbyPasswordLineEdit, &QLineEdit::returnPressed, m_confirmPasswordButton, &QPushButton::click);
    m_confirmPasswordButton->setDefault(true);
    enterLobbyPasswordButtonLayout->addWidget(m_confirmPasswordButton);

    m_cancelJoinLobbyButton = new QPushButton(StringLibrary::getTranslatedString("Cancel"));
    m_cancelJoinLobbyButton->setFixedWidth(MENU_BUTTON_WIDTH);
    connect(m_cancelJoinLobbyButton, &QPushButton::released, this,
            &LobbyPasswordFrame::handleCancelButton);
    enterLobbyPasswordButtonLayout->addWidget(m_cancelJoinLobbyButton);

    enterLobbyPasswordLayout->addLayout(enterLobbyPasswordButtonLayout);
}

LobbyPasswordFrame::~LobbyPasswordFrame()
{
}

void LobbyPasswordFrame::showEvent(QShowEvent *event)
{
    (void)event; // NO LINT

    m_lobbyNameLabel->setText(Parameters::getLobbyName().c_str());
    std::string lobbyIdWithHash = "#" + Parameters::getLobbyId();
    m_lobbyIdLabel->setText(lobbyIdWithHash.c_str());

    m_lobbyPasswordLineEdit->clear();
    m_confirmPasswordButton->setEnabled(false);
    m_passwordIncorrectLabel->setVisible(false);

    m_lobbyPasswordLineEdit->setFocus();

    m_languageSelector->setCurrentIndex(static_cast<int>(Parameters::getLanguage()));
    reloadStrings();
}

void LobbyPasswordFrame::setPasswordIncorrect()
{
    m_passwordIncorrectLabel->setVisible(true);
    m_confirmPasswordButton->setEnabled(false);
}

void LobbyPasswordFrame::validatePasswordText(const QString &lobbyPassword)
{
    m_confirmPasswordButton->setEnabled(!lobbyPassword.isEmpty());
    m_lobbyPasswordLineEdit->setProperty("in_use", !lobbyPassword.isEmpty());
    // Update the style
    m_lobbyPasswordLineEdit->style()->unpolish(m_lobbyPasswordLineEdit);
    m_lobbyPasswordLineEdit->style()->polish(m_lobbyPasswordLineEdit);
    m_lobbyPasswordLineEdit->update();
    m_confirmPasswordButton->setEnabled(!lobbyPassword.isEmpty());
    m_passwordIncorrectLabel->setVisible(false);
}

void LobbyPasswordFrame::handleCancelButton()
{
    m_playerWindow->moveToLobbyListFrame();
}

void LobbyPasswordFrame::handleConfirmPasswordButton()
{
    m_playerWindow->joinLobby(m_lobbyPasswordLineEdit->text().toStdString());
}

void LobbyPasswordFrame::reloadStrings()
{
    m_titleWidget->reloadStrings();

    m_lobbyPasswordLabel->setText(StringLibrary::getTranslatedString("Lobby password"));
    m_passwordIncorrectLabel->setText(StringLibrary::getTranslatedString("Incorrect password"));

    m_confirmPasswordButton->setText(StringLibrary::getTranslatedString("Join Lobby"));
    m_cancelJoinLobbyButton->setText(StringLibrary::getTranslatedString("Cancel"));
}