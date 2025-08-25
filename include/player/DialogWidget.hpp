#pragma once

#include <QWidget>
#include <QLabel>
#include <QPushButton>

#include <string>
#include <vector>
#include <functional>

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
    QLabel *m_headerLabel;

    QPushButton *m_confirmButton;
    QPushButton *m_cancelButton;

    std::string m_headerText;
    std::string m_confirmText;
    std::string m_cancelText;
};