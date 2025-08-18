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

#include "player/CheckersPlayerWindow.hpp"
#include "shared/CheckersConsts.hpp"
#include "player/Parameters.hpp"
#include "player/StringLibrary.hpp"
#include "player/TitleWidget.hpp"
#include "player/LanguageSelectorWidget.hpp"

CreateAccountFrame::CreateAccountFrame(
    CheckersPlayerWindow *parentWindow)
    : QFrame(parentWindow, Qt::WindowFlags())
{
    m_playerWindow = parentWindow;

    m_playerNameValid = false;
    m_playerPasswordValid = false;
    m_creatingAccount = false;

    auto createAccountLayout = new QVBoxLayout(this);
    createAccountLayout->setAlignment(Qt::AlignCenter);

    m_languageSelector = new LanguageSelectorWidget(this);

    m_titleWidget = new TitleWidget();
    createAccountLayout->addWidget(m_titleWidget);

    m_playerNameLabel = new QLabel(StringLibrary::getTranslatedString("Player name"));
    createAccountLayout->addWidget(m_playerNameLabel);

    m_playerNameLineEdit = new QLineEdit();
    m_playerNameLineEdit->setFixedWidth(MENU_LINE_EDIT_WIDTH);
    std::string playerNameRegex = "^[a-zA-Z][a-zA-Z0-9_]{0," + std::to_string(MAX_CHARS_PLAYER_NAME) + "}$";
    auto playerNameValidator = new QRegularExpressionValidator(QRegularExpression(playerNameRegex.c_str()));
    m_playerNameLineEdit->setValidator(playerNameValidator);
    m_playerNameLineEdit->setProperty("valid", false);
    connect(m_playerNameLineEdit, &QLineEdit::textChanged, this, &CreateAccountFrame::validatePlayerNameText);
    createAccountLayout->addWidget(m_playerNameLineEdit);

    m_playerPasswordLabel = new QLabel(StringLibrary::getTranslatedString("Player password"));
    createAccountLayout->addWidget(m_playerPasswordLabel);

    m_passwordLineEdit = new QLineEdit();
    m_passwordLineEdit->setFixedWidth(MENU_LINE_EDIT_WIDTH);
    std::string playerPasswordRegex = "^[a-zA-Z0-9a-zA-Z0-9\\`\\~\\!\\@\\#\\$\\%\\^\\&\\*\\(\\)\\-\\_\\=\\+\\[\\]\\{\\}\\|\\;\\:\\,\\.\\<\\>\\?]{0," + std::to_string(MAX_CHARS_PLAYER_PASS) + "}$";
    auto playerPasswordValidator = new QRegularExpressionValidator(QRegularExpression(playerPasswordRegex.c_str()));
    m_passwordLineEdit->setValidator(playerPasswordValidator);
    m_passwordLineEdit->setEchoMode(QLineEdit::Password);
    m_playerNameLineEdit->setProperty("valid", false);
    connect(m_passwordLineEdit, &QLineEdit::textChanged, this, &CreateAccountFrame::validatePasswordText);
    createAccountLayout->addWidget(m_passwordLineEdit);

    m_passwordWarningLabel1 = new QLabel(StringLibrary::getTranslatedString("This application is a learning project for me, thus passwords are NOT robustly secured."));
    m_passwordWarningLabel2 = new QLabel(StringLibrary::getTranslatedString("Do NOT use passwords you use elsewhere."));
    m_passwordWarningLabel3 = new QLabel(StringLibrary::getTranslatedString("To prevent password reuse, passwords can be a maximum of 3 characters in length."));
    m_passwordWarningLabel4 = new QLabel(StringLibrary::getTranslatedString("You may use alphanumeric characters and/or the following symbols:"));
    m_passwordWarningLabel5 = new QLabel("\t` ~ ! @ # $ % ^ & * ( ) - _ = + [ ] { } | ; : , . < > ?");
    m_passwordWarningLabel6 = new QLabel(StringLibrary::getTranslatedString("Warning: Lost passwords cannot be recovered or changed!"));
    createAccountLayout->addWidget(m_passwordWarningLabel1);
    createAccountLayout->addWidget(m_passwordWarningLabel2);
    createAccountLayout->addWidget(m_passwordWarningLabel3);
    createAccountLayout->addWidget(m_passwordWarningLabel4);
    createAccountLayout->addWidget(m_passwordWarningLabel5);
    createAccountLayout->addWidget(m_passwordWarningLabel6);

    // Add a spacer
    createAccountLayout->addWidget(new QLabel(""));

    m_errorMessageLabel = new QLabel("");
    m_errorMessageLabel->setProperty("error", true);
    auto errorMessageLabelSizePolicy = m_errorMessageLabel->sizePolicy();
    errorMessageLabelSizePolicy.setRetainSizeWhenHidden(true);
    m_errorMessageLabel->setSizePolicy(errorMessageLabelSizePolicy);
    m_errorMessageLabel->setVisible(false);
    createAccountLayout->addWidget(m_errorMessageLabel);

    // Add a spacer
    createAccountLayout->addWidget(new QLabel(""));

    auto createAccountButtonLayout = new QHBoxLayout();
    createAccountButtonLayout->setAlignment(Qt::AlignCenter);

    m_createAccountButton = new QPushButton(StringLibrary::getTranslatedString("Create Account"));
    m_createAccountButton->setFixedWidth(MENU_BUTTON_WIDTH);
    m_createAccountButton->setEnabled(false);
    connect(m_createAccountButton, &QPushButton::released, this,
            &CreateAccountFrame::handleCreateAccountButton);
    m_createAccountButton->setDefault(true);
    connect(m_playerNameLineEdit, &QLineEdit::returnPressed, m_createAccountButton, &QPushButton::click);
    connect(m_passwordLineEdit, &QLineEdit::returnPressed, m_createAccountButton, &QPushButton::click);
    createAccountButtonLayout->addWidget(m_createAccountButton);

    m_cancelButton = new QPushButton(StringLibrary::getTranslatedString("Cancel"));
    m_cancelButton->setFixedWidth(MENU_BUTTON_WIDTH);
    connect(m_cancelButton, &QPushButton::released, this,
            &CreateAccountFrame::handleCancelButton);
    createAccountButtonLayout->addWidget(m_cancelButton);

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
    m_creatingAccount = false;
    m_playerNameLineEdit->clear();
    m_playerNameLineEdit->setProperty("valid", false);
    m_playerNameLineEdit->style()->unpolish(m_playerNameLineEdit);
    m_playerNameLineEdit->style()->polish(m_playerNameLineEdit);
    m_playerNameLineEdit->update();
    m_passwordLineEdit->clear();
    m_playerNameLineEdit->setEnabled(true);
    m_passwordLineEdit->setProperty("valid", false);
    m_passwordLineEdit->style()->unpolish(m_passwordLineEdit);
    m_passwordLineEdit->style()->polish(m_passwordLineEdit);
    m_passwordLineEdit->update();
    m_passwordLineEdit->setEnabled(true);
    m_createAccountButton->setEnabled(false);
    m_errorMessageLabel->setVisible(false);

    m_playerNameLineEdit->setFocus();

    m_languageSelector->setCurrentIndex(static_cast<int>(Parameters::getLanguage()));
    reloadStrings();
}

void CreateAccountFrame::failedCreate(const std::string &errorMessage)
{
    m_errorMessage = errorMessage;
    m_createAccountButton->setText(StringLibrary::getTranslatedString("Create Account"));
    m_errorMessageLabel->setText(StringLibrary::getTranslatedString(m_errorMessage));
    m_errorMessageLabel->setVisible(true);
    m_playerNameLineEdit->setEnabled(true);
    m_passwordLineEdit->setEnabled(true);
    m_creatingAccount = false;
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
    m_creatingAccount = true;
    m_createAccountButton->setText(StringLibrary::getTranslatedString("Creating..."));
    m_createAccountButton->setEnabled(false);
    m_playerNameLineEdit->setEnabled(false);
    m_passwordLineEdit->setEnabled(false);

    auto playerName = m_playerNameLineEdit->text().toStdString();
    auto playerPassword = m_passwordLineEdit->text().toStdString();
    m_playerWindow->createAccount(playerName, playerPassword);
}

void CreateAccountFrame::handleCancelButton()
{
    m_playerWindow->moveToTitleFrame();
}

void CreateAccountFrame::reloadStrings()
{
    m_titleWidget->reloadStrings();

    m_playerNameLabel->setText(StringLibrary::getTranslatedString("Player name"));
    m_playerPasswordLabel->setText(StringLibrary::getTranslatedString("Player password"));

    m_passwordWarningLabel1->setText(StringLibrary::getTranslatedString("This application is a learning project for me, thus passwords are NOT robustly secured."));
    m_passwordWarningLabel2->setText(StringLibrary::getTranslatedString("Do NOT use passwords you use elsewhere."));
    m_passwordWarningLabel3->setText(StringLibrary::getTranslatedString("To prevent password reuse, passwords can be a maximum of 3 characters in length."));
    m_passwordWarningLabel4->setText(StringLibrary::getTranslatedString("You may use alphanumeric characters and/or the following symbols:"));
    // m_passwordWarningLabel5 does not need translating
    m_passwordWarningLabel6->setText(StringLibrary::getTranslatedString("Warning: Lost passwords cannot be recovered or changed!"));

    m_errorMessageLabel->setText(StringLibrary::getTranslatedString(m_errorMessage));

    m_createAccountButton->setText((m_creatingAccount) ? StringLibrary::getTranslatedString("Creating...") : StringLibrary::getTranslatedString("Create Account"));
    m_cancelButton->setText(StringLibrary::getTranslatedString("Cancel"));
}