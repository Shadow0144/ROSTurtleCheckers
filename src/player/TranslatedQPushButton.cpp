#include "player/TranslatedQPushButton.hpp"

#include <QPushButton>
#include <QString>

#include <string>
#include <vector>

#include "player/StringLibrary.hpp"

TranslatedQPushButton::TranslatedQPushButton()
{
}

TranslatedQPushButton::TranslatedQPushButton(const std::string &keyString,
                                             const std::vector<std::string> &parameters)
{
    setText(keyString, parameters);
}

void TranslatedQPushButton::setText(const std::string &keyString,
                                    const std::vector<std::string> &parameters)
{
    m_keyString = keyString;
    m_parameters = parameters;
    reloadStrings();
}

void TranslatedQPushButton::reloadStrings()
{
    QPushButton::setText(StringLibrary::getTranslatedString(m_keyString, m_parameters));
}