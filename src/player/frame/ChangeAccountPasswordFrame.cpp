#include "player/frame/ChangeAccountPasswordFrame.hpp"

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

ChangeAccountPasswordFrame::ChangeAccountPasswordFrame(
    CheckersPlayerWindow *parentWindow)
    : QFrame(parentWindow, Qt::WindowFlags())
{
    m_playerWindow = parentWindow;

    m_previousPlayerPasswordValid = false;
    m_newPlayerPasswordValid = false;
    m_changingPassword = false;

    auto changeAccountPasswordLayout = new QVBoxLayout(this);
    changeAccountPasswordLayout->setAlignment(Qt::AlignCenter);

    m_languageSelector = new LanguageSelectorWidget(this);

    m_titleWidget = new TitleWidget();
    changeAccountPasswordLayout->addWidget(m_titleWidget);

    m_playerNameLabel = new QLabel("");
    auto playerNameFont = m_playerNameLabel->font();
    playerNameFont.setPointSize(PLAYER_NAME_FONT_SIZE);
    m_playerNameLabel->setFont(playerNameFont);
    changeAccountPasswordLayout->addWidget(m_playerNameLabel);

    m_currentPlayerPasswordLabel = new QLabel(StringLibrary::getTranslatedString("Current password"));
    changeAccountPasswordLayout->addWidget(m_currentPlayerPasswordLabel);

    std::string playerPasswordRegex = "^[a-zA-Z0-9a-zA-Z0-9\\`\\~\\!\\@\\#\\$\\%\\^\\&\\*\\(\\)\\-\\_\\=\\+\\[\\]\\{\\}\\|\\;\\:\\,\\.\\<\\>\\?]{0," + std::to_string(MAX_CHARS_PLAYER_PASS) + "}$";
    auto playerPasswordValidator = new QRegularExpressionValidator(QRegularExpression(playerPasswordRegex.c_str()));

    m_previousPasswordLineEdit = new QLineEdit();
    m_previousPasswordLineEdit->setFixedWidth(MENU_LINE_EDIT_WIDTH);
    m_previousPasswordLineEdit->setValidator(playerPasswordValidator);
    m_previousPasswordLineEdit->setEchoMode(QLineEdit::Password);
    m_previousPasswordLineEdit->setProperty("valid", false);
    connect(m_previousPasswordLineEdit, &QLineEdit::textChanged, this, &ChangeAccountPasswordFrame::validatePreviousPasswordText);
    changeAccountPasswordLayout->addWidget(m_previousPasswordLineEdit);

    m_newPlayerPasswordLabel = new QLabel(StringLibrary::getTranslatedString("New password"));
    changeAccountPasswordLayout->addWidget(m_newPlayerPasswordLabel);

    m_newPasswordLineEdit = new QLineEdit();
    m_newPasswordLineEdit->setFixedWidth(MENU_LINE_EDIT_WIDTH);
    m_newPasswordLineEdit->setValidator(playerPasswordValidator);
    m_newPasswordLineEdit->setEchoMode(QLineEdit::Password);
    m_newPasswordLineEdit->setProperty("valid", false);
    connect(m_newPasswordLineEdit, &QLineEdit::textChanged, this, &ChangeAccountPasswordFrame::validateNewPasswordText);
    changeAccountPasswordLayout->addWidget(m_newPasswordLineEdit);

    m_passwordWarningLabel1 = new QLabel(StringLibrary::getTranslatedString("This application is a learning project for me, thus passwords are NOT robustly secured."));
    m_passwordWarningLabel2 = new QLabel(StringLibrary::getTranslatedString("Do NOT use passwords you use elsewhere."));
    m_passwordWarningLabel3 = new QLabel(StringLibrary::getTranslatedString("To prevent password reuse, passwords can be a maximum of 3 characters in length."));
    m_passwordWarningLabel4 = new QLabel(StringLibrary::getTranslatedString("You may use alphanumeric characters and/or the following symbols:"));
    m_passwordWarningLabel5 = new QLabel("\t` ~ ! @ # $ % ^ & * ( ) - _ = + [ ] { } | ; : , . < > ?");
    m_passwordWarningLabel6 = new QLabel(StringLibrary::getTranslatedString("Warning: Lost passwords cannot be recovered or changed!"));
    m_passwordWarningLabel6->setWordWrap(true);
    changeAccountPasswordLayout->addWidget(m_passwordWarningLabel1);
    changeAccountPasswordLayout->addWidget(m_passwordWarningLabel2);
    changeAccountPasswordLayout->addWidget(m_passwordWarningLabel3);
    changeAccountPasswordLayout->addWidget(m_passwordWarningLabel4);
    changeAccountPasswordLayout->addWidget(m_passwordWarningLabel5);
    changeAccountPasswordLayout->addWidget(m_passwordWarningLabel6);

    // Add a spacer
    changeAccountPasswordLayout->addWidget(new QLabel(""));

    m_messageLabel = new QLabel("");
    m_messageLabel->setProperty("succeeded", false);
    m_messageLabel->setProperty("error", false);
    auto errorMessageLabelSizePolicy = m_messageLabel->sizePolicy();
    errorMessageLabelSizePolicy.setRetainSizeWhenHidden(true);
    m_messageLabel->setSizePolicy(errorMessageLabelSizePolicy);
    m_messageLabel->setVisible(false);
    changeAccountPasswordLayout->addWidget(m_messageLabel);

    // Add a spacer
    changeAccountPasswordLayout->addWidget(new QLabel(""));

    auto createAccountButtonLayout = new QHBoxLayout();
    createAccountButtonLayout->setAlignment(Qt::AlignCenter);

    m_changeAccountPasswordButton = new QPushButton(StringLibrary::getTranslatedString("Change Password"));
    m_changeAccountPasswordButton->setFixedWidth(MENU_BUTTON_WIDTH);
    m_changeAccountPasswordButton->setEnabled(false);
    connect(m_changeAccountPasswordButton, &QPushButton::released, this,
            &ChangeAccountPasswordFrame::handleChangeAccountPasswordButton);
    m_changeAccountPasswordButton->setDefault(true);
    connect(m_previousPasswordLineEdit, &QLineEdit::returnPressed, m_changeAccountPasswordButton, &QPushButton::click);
    connect(m_newPasswordLineEdit, &QLineEdit::returnPressed, m_changeAccountPasswordButton, &QPushButton::click);
    createAccountButtonLayout->addWidget(m_changeAccountPasswordButton);

    m_cancelButton = new QPushButton(StringLibrary::getTranslatedString("Cancel"));
    m_cancelButton->setFixedWidth(MENU_BUTTON_WIDTH);
    connect(m_cancelButton, &QPushButton::released, this,
            &ChangeAccountPasswordFrame::handleCancelButton);
    createAccountButtonLayout->addWidget(m_cancelButton);

    changeAccountPasswordLayout->addLayout(createAccountButtonLayout);
}

ChangeAccountPasswordFrame::~ChangeAccountPasswordFrame()
{
}

void ChangeAccountPasswordFrame::showEvent(QShowEvent *event)
{
    (void)event; // NO LINT

    m_playerNameLabel->setText(QString::fromStdString(Parameters::getPlayerName()));

    m_previousPlayerPasswordValid = false;
    m_previousPasswordLineEdit->clear();
    m_previousPasswordLineEdit->setProperty("valid", false);
    m_previousPasswordLineEdit->style()->unpolish(m_previousPasswordLineEdit);
    m_previousPasswordLineEdit->style()->polish(m_previousPasswordLineEdit);
    m_previousPasswordLineEdit->update();

    m_newPlayerPasswordValid = false;
    m_newPasswordLineEdit->clear();
    m_newPasswordLineEdit->setProperty("valid", false);
    m_newPasswordLineEdit->style()->unpolish(m_newPasswordLineEdit);
    m_newPasswordLineEdit->style()->polish(m_newPasswordLineEdit);
    m_newPasswordLineEdit->update();

    m_changingPassword = false;
    m_changeAccountPasswordButton->setText(StringLibrary::getTranslatedString("Change Password"));
    m_previousPasswordLineEdit->setEnabled(true);
    m_newPasswordLineEdit->setEnabled(true);

    m_changeAccountPasswordButton->setEnabled(false);
    m_messageLabel->setVisible(false);

    m_previousPasswordLineEdit->setFocus();
}

void ChangeAccountPasswordFrame::succeededChange()
{
    m_resultMessage = "Password changed successfully";
    m_changingPassword = false;
    m_changeAccountPasswordButton->setText(StringLibrary::getTranslatedString("Change Password"));
    m_previousPasswordLineEdit->setEnabled(true);
    m_newPasswordLineEdit->setEnabled(true);
    m_messageLabel->setText(StringLibrary::getTranslatedString(m_resultMessage));
    m_messageLabel->setProperty("succeeded", true);
    m_messageLabel->setProperty("error", false);
    m_messageLabel->style()->unpolish(m_messageLabel);
    m_messageLabel->style()->polish(m_messageLabel);
    m_messageLabel->update();
    m_messageLabel->setVisible(true);
}

void ChangeAccountPasswordFrame::failedChange(const std::string &errorMessage)
{
    m_resultMessage = errorMessage;
    m_changingPassword = false;
    m_changeAccountPasswordButton->setText(StringLibrary::getTranslatedString("Change Password"));
    m_previousPasswordLineEdit->setEnabled(true);
    m_newPasswordLineEdit->setEnabled(true);
    m_messageLabel->setText(errorMessage.c_str());
    m_messageLabel->setProperty("succeeded", false);
    m_messageLabel->setProperty("error", true);
    m_messageLabel->style()->unpolish(m_messageLabel);
    m_messageLabel->style()->polish(m_messageLabel);
    m_messageLabel->update();
    m_messageLabel->setVisible(true);
}

void ChangeAccountPasswordFrame::validatePreviousPasswordText(const QString &previousPlayerPassword)
{
    QString previousPlayerPasswordCopy = previousPlayerPassword; // Remove the const
    int pos = 0;
    QValidator::State state = m_previousPasswordLineEdit->validator()->validate(previousPlayerPasswordCopy, pos);
    if (!previousPlayerPassword.isEmpty() && state == QValidator::Acceptable)
    {
        m_previousPasswordLineEdit->setProperty("valid", true);
        m_previousPlayerPasswordValid = true;
    }
    else if (previousPlayerPassword.isEmpty() || state == QValidator::Invalid)
    {
        m_previousPasswordLineEdit->setProperty("valid", false);
        m_previousPlayerPasswordValid = false;
    }
    else // Intermediate
    {
        // Do nothing
    }
    m_changeAccountPasswordButton->setEnabled(m_previousPlayerPasswordValid && m_newPlayerPasswordValid);
    m_messageLabel->setVisible(false);
    // Update the style
    m_previousPasswordLineEdit->style()->unpolish(m_previousPasswordLineEdit);
    m_previousPasswordLineEdit->style()->polish(m_previousPasswordLineEdit);
    m_previousPasswordLineEdit->update();
}

void ChangeAccountPasswordFrame::validateNewPasswordText(const QString &newPlayerPassword)
{
    QString newPlayerPasswordCopy = newPlayerPassword; // Remove the const
    int pos = 0;
    QValidator::State state = m_newPasswordLineEdit->validator()->validate(newPlayerPasswordCopy, pos);
    if (!newPlayerPassword.isEmpty() && state == QValidator::Acceptable)
    {
        m_newPasswordLineEdit->setProperty("valid", true);
        m_newPlayerPasswordValid = true;
    }
    else if (newPlayerPassword.isEmpty() || state == QValidator::Invalid)
    {
        m_newPasswordLineEdit->setProperty("valid", false);
        m_newPlayerPasswordValid = false;
    }
    else // Intermediate
    {
        // Do nothing
    }
    m_changeAccountPasswordButton->setEnabled(m_newPlayerPasswordValid && m_newPlayerPasswordValid);
    m_messageLabel->setVisible(false);
    // Update the style
    m_newPasswordLineEdit->style()->unpolish(m_newPasswordLineEdit);
    m_newPasswordLineEdit->style()->polish(m_newPasswordLineEdit);
    m_newPasswordLineEdit->update();
}

void ChangeAccountPasswordFrame::handleChangeAccountPasswordButton()
{
    m_changingPassword = true;
    m_changeAccountPasswordButton->setText(StringLibrary::getTranslatedString("Changing..."));
    m_changeAccountPasswordButton->setEnabled(false);
    m_previousPasswordLineEdit->setEnabled(false);
    m_newPasswordLineEdit->setEnabled(false);

    auto previousPlayerPassword = m_previousPasswordLineEdit->text().toStdString();
    auto newPlayerPassword = m_newPasswordLineEdit->text().toStdString();
    m_playerWindow->changeAccountPassword(previousPlayerPassword, newPlayerPassword);
}

void ChangeAccountPasswordFrame::handleCancelButton()
{
    m_playerWindow->moveToMainMenuFrame();
}

void ChangeAccountPasswordFrame::reloadStrings()
{
    m_titleWidget->reloadStrings();

    m_currentPlayerPasswordLabel->setText(StringLibrary::getTranslatedString("Current password"));
    m_newPlayerPasswordLabel->setText(StringLibrary::getTranslatedString("New password"));

    m_passwordWarningLabel1->setText(StringLibrary::getTranslatedString("This application is a learning project for me, thus passwords are NOT robustly secured."));
    m_passwordWarningLabel2->setText(StringLibrary::getTranslatedString("Do NOT use passwords you use elsewhere."));
    m_passwordWarningLabel3->setText(StringLibrary::getTranslatedString("To prevent password reuse, passwords can be a maximum of 3 characters in length."));
    m_passwordWarningLabel4->setText(StringLibrary::getTranslatedString("You may use alphanumeric characters and/or the following symbols:"));
    // m_passwordWarningLabel5 does not need translating
    m_passwordWarningLabel6->setText(StringLibrary::getTranslatedString("Warning: Lost passwords cannot be recovered or changed!"));

    m_messageLabel->setText(StringLibrary::getTranslatedString(m_resultMessage));

    m_changeAccountPasswordButton->setText((m_changingPassword) ? StringLibrary::getTranslatedString("Changing...") : StringLibrary::getTranslatedString("Change Password"));
    m_cancelButton->setText(StringLibrary::getTranslatedString("Cancel"));
}