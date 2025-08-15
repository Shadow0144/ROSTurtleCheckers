#include "player/LanguageSelectorWidget.hpp"

#include <QComboBox>
#include <QVariant>
#include <QString>
#include <QFont>
#include <QPixmap>
#include <QIcon>

#include <string>
#include <functional>

#include "shared/CheckersConsts.hpp"
#include "player/ImageLibrary.hpp"
#include "player/Parameters.hpp"

LanguageSelectorWidget::LanguageSelectorWidget(QWidget *parent)
    : QWidget(parent)
{
    auto languageComboBox = new QComboBox(this);
    languageComboBox->setIconSize(QSize(LANGUAGE_ICON_WIDTH + 6, LANGUAGE_ICON_HEIGHT + 10));
    auto flagUSIcon = QPixmap::fromImage(ImageLibrary::getFlagUSImage());
    auto scaledFlagUSIcon = QIcon(flagUSIcon.scaled(LANGUAGE_ICON_WIDTH, LANGUAGE_ICON_HEIGHT,
                                                    Qt::KeepAspectRatio, Qt::SmoothTransformation));
    languageComboBox->addItem(scaledFlagUSIcon, "  ");
    auto flagJPIcon = QPixmap::fromImage(ImageLibrary::getFlagJPImage());
    auto scaledFlagJPIcon = QIcon(flagJPIcon.scaled(LANGUAGE_ICON_WIDTH, LANGUAGE_ICON_HEIGHT,
                                                    Qt::KeepAspectRatio, Qt::SmoothTransformation));
    languageComboBox->addItem(scaledFlagJPIcon, "  ");
    auto flagDEIcon = QPixmap::fromImage(ImageLibrary::getFlagDEImage());
    auto scaledFlagDEIcon = QIcon(flagDEIcon.scaled(LANGUAGE_ICON_WIDTH, LANGUAGE_ICON_HEIGHT,
                                                    Qt::KeepAspectRatio, Qt::SmoothTransformation));
    languageComboBox->addItem(scaledFlagDEIcon, "  ");
    languageComboBox->setCurrentIndex(static_cast<int>(Parameters::getLanguage()));
    connect(languageComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &LanguageSelectorWidget::onLanguageSelect);

    move(LANGUAGE_SELECT_X, LANGUAGE_SELECT_Y);
}

void LanguageSelectorWidget::onLanguageSelect(int index)
{
    switch (index)
    {
    case 0:
    {
        Parameters::setLanguage(Language::English);
    }
    break;
    case 1:
    {
        Parameters::setLanguage(Language::Japanese);
    }
    break;
    case 2:
    {
        Parameters::setLanguage(Language::German);
    }
    break;
    default:
    {
        // Unknown language selected
    }
    break;
    }
}