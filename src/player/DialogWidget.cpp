#include "player/DialogWidget.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QVariant>
#include <QPushButton>
#include <QString>
#include <QStyle>

#include <string>
#include <vector>
#include <functional>

#include "shared/CheckersConsts.hpp"

DialogWidget::DialogWidget(QWidget *parent,
                           int centerX,
                           int centerY,
                           const std::string &headerString,
                           const std::string &confirmText,
                           const std::function<void()> &onConfirmFunction,
                           const std::string &cancelText,
                           const std::function<void()> &onCancelFunction)
    : QWidget(parent)
{
    setProperty("dialog", QVariant(true));
    auto dialogLayout = new QVBoxLayout();
    setLayout(dialogLayout);

    if (!headerString.empty())
    {
        auto headerLabel = new QLabel(QString::fromStdString(headerString));
        headerLabel->setAlignment(Qt::AlignCenter);
        headerLabel->setContentsMargins(0, 5, 0, 5);
        dialogLayout->addWidget(headerLabel);
    }

    if (!confirmText.empty() || !cancelText.empty())
    {
        auto buttonsLayoutWidget = new QWidget();
        auto buttonsLayout = new QHBoxLayout();
        buttonsLayoutWidget->setLayout(buttonsLayout);

        if (!confirmText.empty())
        {
            auto confirmButton = new QPushButton();
            confirmButton->setText(QString::fromStdString(confirmText));
            confirmButton->setFixedWidth(MENU_BUTTON_WIDTH);
            connect(confirmButton, &QPushButton::released, this, onConfirmFunction);
            buttonsLayout->addWidget(confirmButton);
        }

        if (!cancelText.empty())
        {
            auto cancelButton = new QPushButton();
            cancelButton->setText(QString::fromStdString(cancelText));
            cancelButton->setFixedWidth(MENU_BUTTON_WIDTH);
            connect(cancelButton, &QPushButton::released, this, onCancelFunction);
            buttonsLayout->addWidget(cancelButton);
        }

        dialogLayout->addWidget(buttonsLayoutWidget);
    }

    dialogLayout->invalidate();
    dialogLayout->activate();
    adjustSize();
    move(centerX - (width() / 2), centerY - (height() / 2));
    hide(); // Hidden by default
}