#pragma once

#include <QWidget>
#include <QLabel>
#include <QLineEdit>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "shared/CheckersConsts.hpp"

class ChatBox : public QWidget
{
public:
    ChatBox(QWidget *parent,
            const std::function<void(const std::string &)> &sendMessageCallback);

    void addMessage(const std::string &playerName,
                    TurtlePieceColor playerColor,
                    const std::string &chatMessage,
                    std::chrono::time_point<std::chrono::system_clock> timeStamp);
    void clear();

private:
    void sendMessage();

    std::function<void(const std::string &)> m_sendMessageCallback;

    QLabel *m_chatContentLabel;
    QLineEdit *m_chatEntryLineEdit;
};