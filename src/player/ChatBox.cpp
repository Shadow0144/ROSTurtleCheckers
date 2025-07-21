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

#include <chrono>
#include <functional>
#include <iomanip>
#include <iostream>

#include "shared/CheckersConsts.hpp"

ChatBox::ChatBox(QWidget *parent,
                 const std::function<void(const std::string &)> &sendMessageCallback)
    : QWidget(parent, Qt::WindowFlags())
{
    m_sendMessageCallback = sendMessageCallback;

    setGeometry(CHAT_BOX_X, 0u, CHAT_BOX_WIDTH, WINDOW_HEIGHT);

    auto chatBoxLayout = new QVBoxLayout(this);
    chatBoxLayout->setAlignment(Qt::AlignTop | Qt::AlignLeft);

    auto chatLabel = new QLabel("Chat");
    auto chatFont = chatLabel->font();
    chatFont.setPointSize(CHAT_HEADER_FONT_SIZE);
    chatLabel->setFont(chatFont);
    chatBoxLayout->addWidget(chatLabel);

    auto chatScrollArea = new QScrollArea();
    chatScrollArea->setWidgetResizable(true);
    chatScrollArea->setFixedSize(CHAT_WIDTH, CHAT_HEIGHT);
    chatScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    chatScrollArea->setObjectName("ChatScrollArea");
    chatBoxLayout->addWidget(chatScrollArea);

    auto chatLayoutWidget = new QWidget();
    chatLayoutWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    auto chatLayout = new QVBoxLayout(chatLayoutWidget);

    auto spacer = new QSpacerItem(0, 0, QSizePolicy::Preferred, QSizePolicy::Expanding);
    chatLayout->addItem(spacer);

    m_chatContentLabel = new QLabel();
    chatLayout->addWidget(m_chatContentLabel);

    chatScrollArea->setWidget(chatLayoutWidget);

    auto chatEntryLayout = new QHBoxLayout();

    m_chatEntryLineEdit = new QLineEdit();
    chatEntryLayout->addWidget(m_chatEntryLineEdit);

    auto sendButton = new QPushButton("Send");
    connect(sendButton, &QPushButton::released, this, &ChatBox::sendMessage);
    connect(m_chatEntryLineEdit, &QLineEdit::returnPressed, sendButton, &QPushButton::click);
    sendButton->setDefault(true);
    chatEntryLayout->addWidget(sendButton);

    chatBoxLayout->addLayout(chatEntryLayout);
}

void ChatBox::sendMessage()
{
    m_sendMessageCallback(m_chatEntryLineEdit->text().toStdString());
    m_chatEntryLineEdit->clear();
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
    messageStringStream << "<br><span style='color:" << colorString << "'>"
                        << playerName
                        << " (" << std::put_time(&tm_now, "%H:%M:%S") << "):</span> "
                        << chatMessage;
    m_chatContentLabel->setText(m_chatContentLabel->text() +
                                QString::fromStdString(messageStringStream.str()));
}

void ChatBox::clear()
{
    m_chatContentLabel->clear();
}