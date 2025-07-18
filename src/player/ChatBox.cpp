#include "player/ChatBox.hpp"

#include <QVBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QSpacerItem>

#include "shared/CheckersConsts.hpp"

ChatBox::ChatBox()
{
    setAlignment(Qt::AlignTop | Qt::AlignLeft);

    auto chatLabel = new QLabel("Chat");
    addWidget(chatLabel);

    auto spacer = new QSpacerItem(0, 0);
    addItem(spacer);

    m_chatBoxLineEdit = new QLineEdit();
    addWidget(m_chatBoxLineEdit);
}

void ChatBox::addChat(const std::string &playerName,
                      const std::string &chatMessage,
                      std::chrono::time_point<std::chrono::system_clock> timeStamp)
{
}

void ChatBox::clear()
{
}