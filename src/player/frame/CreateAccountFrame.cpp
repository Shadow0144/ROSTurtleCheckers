#include "player/frame/CreateAccountFrame.hpp"

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
#include <iostream>

#include "shared/CheckersConsts.hpp"
#include "player/TitleWidget.hpp"
#include "player/CheckersPlayerWindow.hpp"
#include "player/ImageLibrary.hpp"

CreateAccountFrame::CreateAccountFrame(
    CheckersPlayerWindow *parentWindow)
    : QFrame(parentWindow, Qt::WindowFlags())
{
    m_playerWindow = parentWindow;

    m_playerNameValid = false;
    m_playerPasswordValid = false;

    auto createAccountLayout = new QVBoxLayout(this);
    createAccountLayout->setAlignment(Qt::AlignCenter);

    auto titleWidget = new TitleWidget();
    createAccountLayout->addWidget(titleWidget);

    auto playerNameLabel = new QLabel("Player name");
    createAccountLayout->addWidget(playerNameLabel);

    m_playerNameLineEdit = new QLineEdit();
    m_playerNameLineEdit->setFixedWidth(MENU_LINE_EDIT_WIDTH);
    std::string playerNameRegex = "^[a-zA-Z][a-zA-Z0-9_]{0," + std::to_string(MAX_CHARS_PLAYER_NAME) + "}$";
    auto playerNameValidator = new QRegularExpressionValidator(QRegularExpression(playerNameRegex.c_str()));
    m_playerNameLineEdit->setValidator(playerNameValidator);
    m_playerNameLineEdit->setProperty("valid", false);
    connect(m_playerNameLineEdit, &QLineEdit::textChanged, this, &CreateAccountFrame::validatePlayerNameText);
    createAccountLayout->addWidget(m_playerNameLineEdit);

    auto playerPasswordLabel = new QLabel("Player password");
    createAccountLayout->addWidget(playerPasswordLabel);

    m_passwordLineEdit = new QLineEdit();
    m_passwordLineEdit->setFixedWidth(MENU_LINE_EDIT_WIDTH);
    std::string playerPasswordRegex = "^[a-zA-Z0-9a-zA-Z0-9\\`\\~\\!\\@\\#\\$\\%\\^\\&\\*\\(\\)\\-\\_\\=\\+\\[\\]\\{\\}\\|\\;\\:\\,\\.\\<\\>\\?]{0," + std::to_string(MAX_CHARS_PLAYER_PASS) + "}$";
    auto playerPasswordValidator = new QRegularExpressionValidator(QRegularExpression(playerPasswordRegex.c_str()));
    m_passwordLineEdit->setValidator(playerPasswordValidator);
    m_passwordLineEdit->setEchoMode(QLineEdit::Password);
    m_playerNameLineEdit->setProperty("valid", false);
    connect(m_passwordLineEdit, &QLineEdit::textChanged, this, &CreateAccountFrame::validatePasswordText);
    createAccountLayout->addWidget(m_passwordLineEdit);

    auto passwordWarningLabel1 = new QLabel("This application is a learning project for me, thus passwords are NOT robustly secured.");
    auto passwordWarningLabel2 = new QLabel("Do NOT use passwords you use elsewhere.");
    auto passwordWarningLabel3 = new QLabel("To prevent password reuse, passwords can be a maximum of 3 characters in length.");
    auto passwordWarningLabel4 = new QLabel("You may use alphanumeric characters and/or the following symbols:");
    auto passwordWarningLabel5 = new QLabel("\t` ~ ! @ # $ % ^ & * ( ) - _ = + [ ] { } | ; : , . < > ?");
    createAccountLayout->addWidget(passwordWarningLabel1);
    createAccountLayout->addWidget(passwordWarningLabel2);
    createAccountLayout->addWidget(passwordWarningLabel3);
    createAccountLayout->addWidget(passwordWarningLabel4);
    createAccountLayout->addWidget(passwordWarningLabel5);

    m_errorMessageLabel = new QLabel("");
    m_errorMessageLabel->setProperty("error", true);
    auto errorMessageLabelSizePolicy = m_errorMessageLabel->sizePolicy();
    errorMessageLabelSizePolicy.setRetainSizeWhenHidden(true);
    m_errorMessageLabel->setSizePolicy(errorMessageLabelSizePolicy);
    m_errorMessageLabel->setVisible(false);
    createAccountLayout->addWidget(m_errorMessageLabel);

    auto createAccountButtonLayout = new QHBoxLayout();

    std::string createAccountString = "Create Account";
    m_createAccountButton = new QPushButton(createAccountString.c_str());
    m_createAccountButton->setFixedWidth(MENU_BUTTON_WIDTH);
    m_createAccountButton->setEnabled(false);
    connect(m_createAccountButton, &QPushButton::released, this,
            &CreateAccountFrame::handleCreateAccountButton);
    createAccountButtonLayout->addWidget(m_createAccountButton);

    std::string cancelString = "Cancel";
    auto cancelButton = new QPushButton(cancelString.c_str());
    cancelButton->setFixedWidth(MENU_BUTTON_WIDTH);
    connect(cancelButton, &QPushButton::released, this,
            &CreateAccountFrame::handleCancelButton);
    createAccountButtonLayout->addWidget(cancelButton);

    createAccountLayout->addLayout(createAccountButtonLayout);
}

CreateAccountFrame::~CreateAccountFrame()
{
}

void CreateAccountFrame::showEvent(QShowEvent *event)
{
    (void)event; // NO LINT

    m_playerNameValid = false;
    m_playerPasswordValid = false;
    m_playerNameLineEdit->clear();
    m_playerNameLineEdit->setProperty("valid", false);
    m_playerNameLineEdit->style()->unpolish(m_playerNameLineEdit);
    m_playerNameLineEdit->style()->polish(m_playerNameLineEdit);
    m_playerNameLineEdit->update();
    m_passwordLineEdit->clear();
    m_passwordLineEdit->setProperty("valid", false);
    m_passwordLineEdit->style()->unpolish(m_passwordLineEdit);
    m_passwordLineEdit->style()->polish(m_passwordLineEdit);
    m_passwordLineEdit->update();
    std::string createAccountString = "Create Account";
    m_createAccountButton->setText(createAccountString.c_str());
    m_createAccountButton->setEnabled(false);
    m_errorMessageLabel->setVisible(false);
}

void CreateAccountFrame::failedCreate(const std::string &errorMessage)
{
    std::string createAccountString = "Create Account";
    m_createAccountButton->setText(createAccountString.c_str());
    m_errorMessageLabel->setText(errorMessage.c_str());
    m_errorMessageLabel->setVisible(true);
}

void CreateAccountFrame::validatePlayerNameText(const QString &playerName)
{
    QString playerNameCopy = playerName; // Remove the const
    int pos = 0;
    QValidator::State state = m_playerNameLineEdit->validator()->validate(playerNameCopy, pos);
    if (!playerName.isEmpty() && state == QValidator::Acceptable)
    {
        m_playerNameLineEdit->setProperty("valid", true);
        m_playerNameValid = true;
    }
    else if (playerName.isEmpty() || state == QValidator::Invalid)
    {
        m_playerNameLineEdit->setProperty("valid", false);
        m_playerNameValid = false;
    }
    else // Intermediate
    {
        // Do nothing
    }
    m_createAccountButton->setEnabled(m_playerNameValid && m_playerPasswordValid);
    m_errorMessageLabel->setVisible(false);
    // Update the style
    m_playerNameLineEdit->style()->unpolish(m_playerNameLineEdit);
    m_playerNameLineEdit->style()->polish(m_playerNameLineEdit);
    m_playerNameLineEdit->update();
}

void CreateAccountFrame::validatePasswordText(const QString &playerPassword)
{
    QString playerPasswordCopy = playerPassword; // Remove the const
    int pos = 0;
    QValidator::State state = m_passwordLineEdit->validator()->validate(playerPasswordCopy, pos);
    if (!playerPassword.isEmpty() && state == QValidator::Acceptable)
    {
        m_passwordLineEdit->setProperty("valid", true);
        m_playerPasswordValid = true;
    }
    else if (playerPassword.isEmpty() || state == QValidator::Invalid)
    {
        m_passwordLineEdit->setProperty("valid", false);
        m_playerPasswordValid = false;
    }
    else // Intermediate
    {
        // Do nothing
    }
    m_createAccountButton->setEnabled(m_playerNameValid && m_playerPasswordValid);
    m_errorMessageLabel->setVisible(false);
    // Update the style
    m_passwordLineEdit->style()->unpolish(m_passwordLineEdit);
    m_passwordLineEdit->style()->polish(m_passwordLineEdit);
    m_passwordLineEdit->update();
}

void CreateAccountFrame::handleCreateAccountButton()
{
    std::string creatingAccountString = "Creating...";
    m_createAccountButton->setText(creatingAccountString.c_str());
    m_createAccountButton->setEnabled(false);

    auto playerName = m_playerNameLineEdit->text().toStdString();
    auto playerPassword = m_passwordLineEdit->text().toStdString();
    m_playerWindow->createAccount(playerName, playerPassword);
}

void CreateAccountFrame::handleCancelButton()
{
    m_playerWindow->moveToTitleFrame();
}