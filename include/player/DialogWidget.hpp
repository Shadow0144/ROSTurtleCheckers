#pragma once

#include <QWidget>
#include <QLabel>
#include <QPushButton>

#include <string>
#include <vector>
#include <functional>

#include "player/TranslatedQLabel.hpp"
#include "player/TranslatedQPushButton.hpp"

class DialogWidget : public QWidget
{
public:
    DialogWidget(QWidget *parent,
                 int centerX,
                 int centerY,
                 const std::string &headerText,
                 const std::string &confirmText,
                 const std::function<void()> &onConfirmFunction,
                 const std::string &cancelText,
                 const std::function<void()> &onCancelFunction);

    void reloadStrings();

private:
    TranslatedQLabel *m_headerLabel;

    TranslatedQPushButton *m_confirmButton;
    TranslatedQPushButton *m_cancelButton;
};