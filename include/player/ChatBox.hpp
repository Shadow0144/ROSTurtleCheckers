#pragma once

#include <QWidget>
#include <QScrollArea>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

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
            int chatWidth,
            int chatHeight,
            const std::function<void(const std::string &)> &reportPlayerCallback,
            const std::function<void(const std::string &)> &sendMessageCallback);

    void addMessage(const std::string &playerName,
                    TurtlePieceColor playerColor,
                    const std::string &chatMessage,
                    std::chrono::time_point<std::chrono::system_clock> timeStamp);
    void clear();

    void setReportPlayerButtonEnabled(bool isEnabled);

    void reloadStrings();

public slots:
    void validateChatEntryText(const QString &chatEntry);

private:
    void handleReportPlayerButton();
    void handleSendMessageButton();

    std::function<void(const std::string &)> m_reportPlayerCallback;
    std::function<void(const std::string &)> m_sendMessageCallback;

    QScrollArea *m_chatScrollArea;
    QWidget *m_chatLayoutWidget;
    QLabel *m_chatContentLabel;
    QLineEdit *m_chatEntryLineEdit;

    QPushButton *m_reportPlayerButton;
    QPushButton *m_sendButton;
};

typedef std::shared_ptr<ChatBox> ChatBoxPtr;