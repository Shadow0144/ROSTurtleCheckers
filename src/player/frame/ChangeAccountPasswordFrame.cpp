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

#include "shared/CheckersConsts.hpp"
#include "player/Parameters.hpp"
#include "player/TitleWidget.hpp"
#include "player/CheckersPlayerWindow.hpp"
#include "player/ImageLibrary.hpp"

ChangeAccountPasswordFrame::ChangeAccountPasswordFrame(
    CheckersPlayerWindow *parentWindow)
    : QFrame(parentWindow, Qt::WindowFlags())
{
    m_playerWindow = parentWindow;

    m_previousPlayerPasswordValid = false;
    m_newPlayerPasswordValid = false;

    auto changeAccountPasswordLayout = new QVBoxLayout(this);
    changeAccountPasswordLayout->setAlignment(Qt::AlignCenter);

    auto titleWidget = new TitleWidget();
    changeAccountPasswordLayout->addWidget(titleWidget);

    m_playerNameLabel = new QLabel("");
    auto playerNameFont = m_playerNameLabel->font();
    playerNameFont.setPointSize(PLAYER_NAME_FONT_SIZE);
    m_playerNameLabel->setFont(playerNameFont);
    changeAccountPasswordLayout->addWidget(m_playerNameLabel);

    auto previousPlayerPasswordLabel = new QLabel("Previous password");
    changeAccountPasswordLayout->addWidget(previousPlayerPasswordLabel);

    std::string playerPasswordRegex = "^[a-zA-Z0-9a-zA-Z0-9\\`\\~\\!\\@\\#\\$\\%\\^\\&\\*\\(\\)\\-\\_\\=\\+\\[\\]\\{\\}\\|\\;\\:\\,\\.\\<\\>\\?]{0," + std::to_string(MAX_CHARS_PLAYER_PASS) + "}$";
    auto playerPasswordValidator = new QRegularExpressionValidator(QRegularExpression(playerPasswordRegex.c_str()));

    m_previousPasswordLineEdit = new QLineEdit();
    m_previousPasswordLineEdit->setFixedWidth(MENU_LINE_EDIT_WIDTH);
    m_previousPasswordLineEdit->setValidator(playerPasswordValidator);
    m_previousPasswordLineEdit->setEchoMode(QLineEdit::Password);
    m_previousPasswordLineEdit->setProperty("valid", false);
    connect(m_previousPasswordLineEdit, &QLineEdit::textChanged, this, &ChangeAccountPasswordFrame::validatePreviousPasswordText);
    changeAccountPasswordLayout->addWidget(m_previousPasswordLineEdit);

    auto newPlayerPasswordLabel = new QLabel("New password");
    changeAccountPasswordLayout->addWidget(newPlayerPasswordLabel);

    m_newPasswordLineEdit = new QLineEdit();
    m_newPasswordLineEdit->setFixedWidth(MENU_LINE_EDIT_WIDTH);
    m_newPasswordLineEdit->setValidator(playerPasswordValidator);
    m_newPasswordLineEdit->setEchoMode(QLineEdit::Password);
    m_newPasswordLineEdit->setProperty("valid", false);
    connect(m_newPasswordLineEdit, &QLineEdit::textChanged, this, &ChangeAccountPasswordFrame::validateNewPasswordText);
    changeAccountPasswordLayout->addWidget(m_newPasswordLineEdit);

    auto passwordWarningLabel1 = new QLabel("This application is a learning project for me, thus passwords are NOT robustly secured.");
    auto passwordWarningLabel2 = new QLabel("Do NOT use passwords you use elsewhere.");
    auto passwordWarningLabel3 = new QLabel("To prevent password reuse, passwords can be a maximum of 3 characters in length.");
    auto passwordWarningLabel4 = new QLabel("You may use alphanumeric characters and/or the following symbols:");
    auto passwordWarningLabel5 = new QLabel("\t` ~ ! @ # $ % ^ & * ( ) - _ = + [ ] { } | ; : , . < > ?");
    auto passwordWarningLabel6 = new QLabel("There is also no way to recover or change your password if you lose it, so make sure the password is memorable!");
    changeAccountPasswordLayout->addWidget(passwordWarningLabel1);
    changeAccountPasswordLayout->addWidget(passwordWarningLabel2);
    changeAccountPasswordLayout->addWidget(passwordWarningLabel3);
    changeAccountPasswordLayout->addWidget(passwordWarningLabel4);
    changeAccountPasswordLayout->addWidget(passwordWarningLabel5);
    changeAccountPasswordLayout->addWidget(passwordWarningLabel6);

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

    std::string changePasswordString = "Change Password";
    m_changeAccountPasswordButton = new QPushButton(changePasswordString.c_str());
    m_changeAccountPasswordButton->setFixedWidth(MENU_BUTTON_WIDTH);
    m_changeAccountPasswordButton->setEnabled(false);
    connect(m_changeAccountPasswordButton, &QPushButton::released, this,
            &ChangeAccountPasswordFrame::handleChangeAccountPasswordButton);
    m_changeAccountPasswordButton->setDefault(true);
    connect(m_previousPasswordLineEdit, &QLineEdit::returnPressed, m_changeAccountPasswordButton, &QPushButton::click);
    connect(m_newPasswordLineEdit, &QLineEdit::returnPressed, m_changeAccountPasswordButton, &QPushButton::click);
    createAccountButtonLayout->addWidget(m_changeAccountPasswordButton);

    std::string cancelString = "Cancel";
    auto cancelButton = new QPushButton(cancelString.c_str());
    cancelButton->setFixedWidth(MENU_BUTTON_WIDTH);
    connect(cancelButton, &QPushButton::released, this,
            &ChangeAccountPasswordFrame::handleCancelButton);
    createAccountButtonLayout->addWidget(cancelButton);

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

    m_changeAccountPasswordButton->setEnabled(false);
    m_messageLabel->setVisible(false);

    m_previousPasswordLineEdit->setFocus();
}

void ChangeAccountPasswordFrame::succeededChange()
{
    m_messageLabel->setText("Password changed successfully");
    m_messageLabel->setProperty("succeeded", true);
    m_messageLabel->setProperty("error", false);
    m_messageLabel->style()->unpolish(m_messageLabel);
    m_messageLabel->style()->polish(m_messageLabel);
    m_messageLabel->update();
    m_messageLabel->setVisible(true);
}

void ChangeAccountPasswordFrame::failedChange(const std::string &errorMessage)
{
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
    m_changeAccountPasswordButton->setEnabled(false);

    auto previousPlayerPassword = m_previousPasswordLineEdit->text().toStdString();
    auto newPlayerPassword = m_newPasswordLineEdit->text().toStdString();
    m_playerWindow->changeAccountPassword(previousPlayerPassword, newPlayerPassword);
}

void ChangeAccountPasswordFrame::handleCancelButton()
{
    m_playerWindow->moveToMainMenuFrame();
}