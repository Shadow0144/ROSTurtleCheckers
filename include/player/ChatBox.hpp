#pragma once

#include <QVBoxLayout>
#include <QLineEdit>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "shared/CheckersConsts.hpp"

class ChatBox : public QVBoxLayout
{
public:
    ChatBox();

    void addChat(const std::string &playerName,
                 const std::string &chatMessage,
                 std::chrono::time_point<std::chrono::system_clock> timeStamp);
    void clear();

private:
    QLineEdit *m_chatBoxLineEdit;
};

typedef std::shared_ptr<ChatBox> ChatBoxPtr;