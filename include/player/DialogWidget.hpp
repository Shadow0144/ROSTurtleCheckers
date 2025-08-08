#pragma once

#include <QWidget>

#include <string>
#include <vector>
#include <functional>

class DialogWidget : public QWidget
{
public:
    DialogWidget(QWidget *parent,
                 int centerX,
                 int centerY,
                 const std::string &headerString,
                 const std::string &confirmText,
                 const std::function<void()> &onConfirmFunction,
                 const std::string &cancelText,
                 const std::function<void()> &onCancelFunction);
};