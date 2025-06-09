#pragma once

#include <QPainter>
#include <QPen>
#include <QPointF>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "shared/CheckersConsts.hpp"

class ChatBox
{
public:
    ChatBox();

    void addChat(const std::string &playerName,
                 const std::string &chatMessage,
                 std::chrono::time_point<std::chrono::system_clock> timeStamp);
    void clear();

    void paint(QPainter &painter) const;

private:
};

typedef std::shared_ptr<ChatBox> ChatBoxPtr;