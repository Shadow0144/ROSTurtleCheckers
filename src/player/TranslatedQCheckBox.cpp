#include "player/TranslatedQCheckBox.hpp"

#include <QCheckBox>
#include <QString>

#include <string>
#include <vector>

#include "player/StringLibrary.hpp"

TranslatedQCheckBox::TranslatedQCheckBox()
{
}

TranslatedQCheckBox::TranslatedQCheckBox(const std::string &keyString,
                                   const std::vector<std::string> &parameters)
{
    setText(keyString, parameters);
}

void TranslatedQCheckBox::setText(const std::string &keyString,
                               const std::vector<std::string> &parameters)
{
    m_keyString = keyString;
    m_parameters = parameters;
    reloadStrings();
}

void TranslatedQCheckBox::reloadStrings()
{
    QCheckBox::setText(StringLibrary::getTranslatedString(m_keyString, m_parameters));
}