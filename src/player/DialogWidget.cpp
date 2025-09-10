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
#include "player/TranslatedQLabel.hpp"
#include "player/TranslatedQPushButton.hpp"

DialogWidget::DialogWidget(QWidget *parent,
                           int centerX,
                           int centerY,
                           const std::string &headerText,
                           const std::string &confirmText,
                           const std::function<void()> &onConfirmFunction,
                           const std::string &cancelText,
                           const std::function<void()> &onCancelFunction)
    : QWidget(parent)
{
    m_headerLabel = nullptr;
    m_confirmButton = nullptr;
    m_cancelButton = nullptr;

    setProperty("dialog", QVariant(true));
    auto dialogLayout = new QVBoxLayout();
    setLayout(dialogLayout);

    if (!headerText.empty())
    {
        m_headerLabel = new TranslatedQLabel(headerText);
        m_headerLabel->setAlignment(Qt::AlignCenter);
        m_headerLabel->setContentsMargins(0, 5, 0, 5);
        dialogLayout->addWidget(m_headerLabel);
    }
    else
    {
        m_headerLabel = nullptr;
    }

    if (!confirmText.empty() || !cancelText.empty())
    {
        auto buttonsLayoutWidget = new QWidget();
        auto buttonsLayout = new QHBoxLayout();
        buttonsLayoutWidget->setLayout(buttonsLayout);

        if (!confirmText.empty())
        {
            m_confirmButton = new TranslatedQPushButton(confirmText);
            m_confirmButton->setFixedWidth(MENU_BUTTON_WIDTH);
            connect(m_confirmButton, &QPushButton::released, this, onConfirmFunction);
            buttonsLayout->addWidget(m_confirmButton);
        }

        if (!cancelText.empty())
        {
            m_cancelButton = new TranslatedQPushButton(cancelText);
            m_cancelButton->setFixedWidth(MENU_BUTTON_WIDTH);
            connect(m_cancelButton, &QPushButton::released, this, onCancelFunction);
            buttonsLayout->addWidget(m_cancelButton);
        }

        dialogLayout->addWidget(buttonsLayoutWidget);
    }

    dialogLayout->invalidate();
    dialogLayout->activate();
    adjustSize();
    move(centerX - (width() / 2), centerY - (height() / 2));
    hide(); // Hidden by default
}

void DialogWidget::reloadStrings()
{
    if (m_headerLabel)
    {
        m_headerLabel->reloadStrings();
    }

    if (m_confirmButton)
    {
        m_confirmButton->reloadStrings();
    }

    if (m_cancelButton)
    {
        m_cancelButton->reloadStrings();
    }
}