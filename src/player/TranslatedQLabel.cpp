#include "player/TranslatedQLabel.hpp"

#include <QLabel>
#include <QString>

#include <string>
#include <vector>

#include "player/StringLibrary.hpp"

TranslatedQLabel::TranslatedQLabel()
{
}

TranslatedQLabel::TranslatedQLabel(const std::string &keyString,
                                   const std::vector<std::string> &parameters)
{
    setText(keyString, parameters);
}

void TranslatedQLabel::setText(const std::string &keyString,
                               const std::vector<std::string> &parameters)
{
    m_keyString = keyString;
    m_parameters = parameters;
    reloadStrings();
}

void TranslatedQLabel::reloadStrings()
{
    QLabel::setText(StringLibrary::getTranslatedString(m_keyString, m_parameters));
}