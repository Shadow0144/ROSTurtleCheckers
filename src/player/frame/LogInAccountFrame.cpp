#include "player/frame/LogInAccountFrame.hpp"

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

LogInAccountFrame::LogInAccountFrame(
    CheckersPlayerWindow *parentWindow)
    : QFrame(parentWindow, Qt::WindowFlags())
{
    m_playerWindow = parentWindow;

    m_playerNameValid = false;
    m_playerPasswordValid = false;

    auto logInAccountLayout = new QVBoxLayout(this);
    logInAccountLayout->setAlignment(Qt::AlignCenter);

    auto logInAccountTitleLabel = new QLabel("Turtle Checkers");
    auto titleFont = logInAccountTitleLabel->font();
    titleFont.setPointSize(TITLE_FONT_SIZE);
    logInAccountTitleLabel->setFont(titleFont);
    logInAccountLayout->addWidget(logInAccountTitleLabel);

    auto playerNameLabel = new QLabel("Player name");
    logInAccountLayout->addWidget(playerNameLabel);

    m_playerNameLineEdit = new QLineEdit();
    std::string playerNameRegex = "^[a-zA-Z][a-zA-Z0-9_]{0," + std::to_string(MAX_CHARS_PLAYER_NAME) + "}$";
    auto playerNameValidator = new QRegularExpressionValidator(QRegularExpression(playerNameRegex.c_str()));
    m_playerNameLineEdit->setValidator(playerNameValidator);
    m_playerNameLineEdit->setProperty("valid", false);
    connect(m_playerNameLineEdit, &QLineEdit::textChanged, this, &LogInAccountFrame::validatePlayerNameText);
    logInAccountLayout->addWidget(m_playerNameLineEdit);

    auto playerPasswordLabel = new QLabel("Player password");
    logInAccountLayout->addWidget(playerPasswordLabel);

    m_passwordLineEdit = new QLineEdit();
    std::string playerPasswordRegex = "^[a-zA-Z0-9a-zA-Z0-9\\`\\~\\!\\@\\#\\$\\%\\^\\&\\*\\(\\)\\-\\_\\=\\+\\[\\]\\{\\}\\|\\;\\:\\,\\.\\<\\>\\?]{0," + std::to_string(MAX_CHARS_PLAYER_PASS) + "}$";
    auto playerPasswordValidator = new QRegularExpressionValidator(QRegularExpression(playerPasswordRegex.c_str()));
    m_passwordLineEdit->setValidator(playerPasswordValidator);
    m_passwordLineEdit->setEchoMode(QLineEdit::Password);
    m_playerNameLineEdit->setProperty("valid", false);
    connect(m_passwordLineEdit, &QLineEdit::textChanged, this, &LogInAccountFrame::validatePasswordText);
    logInAccountLayout->addWidget(m_passwordLineEdit);

    m_errorMessageLabel = new QLabel("");
    m_errorMessageLabel->setProperty("error", true);
    auto errorMessageLabelSizePolicy = m_errorMessageLabel->sizePolicy();
    errorMessageLabelSizePolicy.setRetainSizeWhenHidden(true);
    m_errorMessageLabel->setSizePolicy(errorMessageLabelSizePolicy);
    m_errorMessageLabel->setVisible(false);
    logInAccountLayout->addWidget(m_errorMessageLabel);

    auto logInAccountButtonLayout = new QHBoxLayout();

    std::string logInAccountString = "Log In";
    m_logInAccountButton = new QPushButton(logInAccountString.c_str());
    m_logInAccountButton->setEnabled(false);
    connect(m_logInAccountButton, &QPushButton::released, this,
            &LogInAccountFrame::handleLogInAccountButton);
    logInAccountButtonLayout->addWidget(m_logInAccountButton);

    std::string cancelString = "Cancel";
    auto cancelButton = new QPushButton(cancelString.c_str());
    connect(cancelButton, &QPushButton::released, this,
            &LogInAccountFrame::handleCancelButton);
    logInAccountButtonLayout->addWidget(cancelButton);

    logInAccountLayout->addLayout(logInAccountButtonLayout);
}

LogInAccountFrame::~LogInAccountFrame()
{
}

void LogInAccountFrame::showEvent(QShowEvent *event)
{
    (void)event; // NO LINT

    m_playerNameValid = false;
    m_playerPasswordValid = false;
    m_playerNameLineEdit->clear();
    m_playerNameLineEdit->setProperty("valid", false);
    m_playerNameLineEdit->style()->unpolish(m_playerNameLineEdit);
    m_playerNameLineEdit->style()->polish(m_playerNameLineEdit);
    m_playerNameLineEdit->update();
    m_playerNameLineEdit->clear();
    m_passwordLineEdit->setProperty("valid", false);
    m_passwordLineEdit->style()->unpolish(m_passwordLineEdit);
    m_passwordLineEdit->style()->polish(m_passwordLineEdit);
    m_passwordLineEdit->update();
    m_errorMessageLabel->setVisible(false);
    m_logInAccountButton->setEnabled(false);
}

void LogInAccountFrame::failedLogIn(const std::string &errorMessage)
{
    std::string logInAccountString = "LogIn";
    m_logInAccountButton->setText(logInAccountString.c_str());
    m_errorMessageLabel->setText(errorMessage.c_str());
    m_errorMessageLabel->setVisible(true);
}

void LogInAccountFrame::validatePlayerNameText(const QString &playerName)
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
    m_logInAccountButton->setEnabled(m_playerNameValid && m_playerPasswordValid);
    m_errorMessageLabel->setVisible(false);
    // Update the style
    m_playerNameLineEdit->style()->unpolish(m_playerNameLineEdit);
    m_playerNameLineEdit->style()->polish(m_playerNameLineEdit);
    m_playerNameLineEdit->update();
}

void LogInAccountFrame::validatePasswordText(const QString &playerPassword)
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
    m_logInAccountButton->setEnabled(m_playerNameValid && m_playerPasswordValid);
    m_errorMessageLabel->setVisible(false);
    // Update the style
    m_passwordLineEdit->style()->unpolish(m_passwordLineEdit);
    m_passwordLineEdit->style()->polish(m_passwordLineEdit);
    m_passwordLineEdit->update();
}

void LogInAccountFrame::handleLogInAccountButton()
{
    std::string creatingAccountString = "Logging in...";
    m_logInAccountButton->setText(creatingAccountString.c_str());
    m_logInAccountButton->setEnabled(false);

    auto playerName = m_playerNameLineEdit->text().toStdString();
    auto playerPassword = m_passwordLineEdit->text().toStdString();
    m_playerWindow->logInAccount(playerName, playerPassword);
}

void LogInAccountFrame::handleCancelButton()
{
    m_playerWindow->moveToTitleFrame();
}