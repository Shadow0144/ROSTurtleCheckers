#include "player/ChatBox.hpp"

#include <QWidget>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QScrollArea>
#include <QLabel>
#include <QLineEdit>
#include <QSpacerItem>
#include <QPushButton>
#include <QString>
#include <QRegularExpression>
#include <QRegularExpressionValidator>
#include <QScrollBar>

#include <chrono>
#include <functional>
#include <iomanip>
#include <iostream>

#include "shared/CheckersConsts.hpp"
#include "player/StringLibrary.hpp"

ChatBox::ChatBox(QWidget *parent,
                 int chatWidth,
                 int chatHeight,
                 const std::function<void(const std::string &)> &reportPlayerCallback,
                 const std::function<void(const std::string &)> &sendMessageCallback)
    : QWidget(parent)
{
    m_reportPlayerCallback = reportPlayerCallback;
    m_sendMessageCallback = sendMessageCallback;

    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    setFixedWidth(chatWidth);
    setFixedHeight(chatHeight);

    auto chatBoxLayout = new QVBoxLayout(this);
    chatBoxLayout->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
    chatBoxLayout->setContentsMargins(0, 0, 0, 0);
    chatBoxLayout->setSpacing(0);

    auto reportPlayerWidget = new QWidget();
    auto reportPlayerLayout = new QHBoxLayout();
    reportPlayerLayout->setContentsMargins(9, 10, 0, 0);
    reportPlayerLayout->setSpacing(0);
    reportPlayerLayout->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    reportPlayerWidget->setLayout(reportPlayerLayout);

    m_reportPlayerButton = new QPushButton(StringLibrary::getTranslatedString("Report Player"));
    m_reportPlayerButton->setFixedWidth(MENU_BUTTON_WIDTH);
    m_reportPlayerButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
    connect(m_reportPlayerButton, &QPushButton::released, this, &ChatBox::handleReportPlayerButton);
    reportPlayerLayout->addWidget(m_reportPlayerButton);

    chatBoxLayout->addWidget(reportPlayerWidget);

    auto boxAndButtonWidget = new QWidget();
    auto boxesAndButtonsLayout = new QVBoxLayout();
    boxesAndButtonsLayout->setSpacing(0);
    boxAndButtonWidget->setLayout(boxesAndButtonsLayout);

    m_chatScrollArea = new QScrollArea();
    m_chatScrollArea->setWidgetResizable(true);
    m_chatScrollArea->setFixedSize(chatWidth - CHAT_BOX_MARGIN_W, chatHeight - CHAT_BOX_MARGIN_H);
    m_chatScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    m_chatScrollArea->setObjectName("ChatScrollArea");
    boxesAndButtonsLayout->addWidget(m_chatScrollArea);

    m_chatLayoutWidget = new QWidget();
    auto chatLayout = new QVBoxLayout(m_chatLayoutWidget);

    auto spacer = new QSpacerItem(0, 0, QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);
    chatLayout->addItem(spacer);

    m_chatContentLabel = new QLabel();
    m_chatContentLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    m_chatContentLabel->setWordWrap(true);
    chatLayout->addWidget(m_chatContentLabel);

    m_chatScrollArea->setWidget(m_chatLayoutWidget);

    auto chatEntryWidget = new QWidget();
    auto chatEntryLayout = new QHBoxLayout();
    chatEntryWidget->setLayout(chatEntryLayout);
    chatEntryWidget->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
    chatEntryLayout->setContentsMargins(0, 9, 0, 9);
    chatEntryLayout->setSpacing(6);

    m_chatEntryLineEdit = new QLineEdit();
    m_chatEntryLineEdit->setMaxLength(MAX_CHARS_CHAT_BOX);
    // Unneeded, but keep in case we want to add it or something like it back in
    // std::string chatRegex = "^[a-zA-Z0-9a-zA-Z0-9 \\'\\\"\\/\\\\\\`\\~\\!\\@\\#\\$\\%\\^\\&\\*\\(\\)\\-\\_\\=\\+\\[\\]\\{\\}\\|\\;\\:\\,\\.\\<\\>\\?]{0," + std::to_string(MAX_CHARS_CHAT_BOX) + "}$";
    // auto chatValidator = new QRegularExpressionValidator(QRegularExpression(chatRegex.c_str()));
    // m_chatEntryLineEdit->setValidator(chatValidator);
    connect(m_chatEntryLineEdit, &QLineEdit::textChanged, this, &ChatBox::validateChatEntryText);

    chatEntryLayout->addWidget(m_chatEntryLineEdit);

    m_sendButton = new QPushButton(StringLibrary::getTranslatedString("Send"));
    m_sendButton->setFixedWidth(MENU_BUTTON_WIDTH);
    m_sendButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::MinimumExpanding);
    connect(m_sendButton, &QPushButton::released, this, &ChatBox::handleSendMessageButton);
    connect(m_chatEntryLineEdit, &QLineEdit::returnPressed, m_sendButton, &QPushButton::click);
    m_sendButton->setDefault(true);
    m_sendButton->setEnabled(false);
    chatEntryLayout->addWidget(m_sendButton);

    boxesAndButtonsLayout->addWidget(chatEntryWidget);

    chatBoxLayout->addWidget(boxAndButtonWidget);
}

void ChatBox::validateChatEntryText(const QString &chatEntry)
{
    m_sendButton->setEnabled(!chatEntry.isEmpty());
}

void ChatBox::handleReportPlayerButton()
{
    m_reportPlayerCallback(m_chatContentLabel->text().toStdString());
}

void ChatBox::handleSendMessageButton()
{
    // Sanitize before sending
    const auto &message = m_chatEntryLineEdit->text().toHtmlEscaped().toStdString();
    m_sendMessageCallback(message);
    m_chatEntryLineEdit->clear();
    m_sendButton->setEnabled(false);
}

void ChatBox::addMessage(const std::string &playerName,
                         TurtlePieceColor playerColor,
                         const std::string &chatMessage,
                         std::chrono::time_point<std::chrono::system_clock> timeStamp)
{
    std::ostringstream messageStringStream;
    std::time_t time_t_now = std::chrono::system_clock::to_time_t(timeStamp);
    std::tm tm_now = *std::localtime(&time_t_now);
    std::string colorString = "white";
    if (playerColor == TurtlePieceColor::Black)
    {
        colorString = "grey";
    }
    else
    {
        colorString = "red";
    }
    if (!m_chatContentLabel->text().isEmpty())
    {
        messageStringStream << "<br>"; // Add a break only if there was a previous line
    }
    messageStringStream << "<span style='color:" << colorString << "'>"
                        << playerName
                        << " (" << std::put_time(&tm_now, "%H:%M:%S") << "):</span><br>&nbsp;"
                        << chatMessage;
    m_chatContentLabel->setText(m_chatContentLabel->text() +
                                QString::fromStdString(messageStringStream.str()));
    m_chatLayoutWidget->adjustSize();
    m_chatScrollArea->verticalScrollBar()->setValue(m_chatScrollArea->verticalScrollBar()->maximum());
}

void ChatBox::clear()
{
    m_chatContentLabel->clear();
}

void ChatBox::setReportPlayerButtonEnabled(bool isEnabled)
{
    m_reportPlayerButton->setEnabled(isEnabled);
}

void ChatBox::reloadStrings()
{
    m_reportPlayerButton->setText(StringLibrary::getTranslatedString("Report Player"));
    m_sendButton->setText(StringLibrary::getTranslatedString("Send"));
}