#include "player/TranslatedQComboBox.hpp"

#include <QComboBox>
#include <QString>

#include <string>
#include <vector>

#include "player/StringLibrary.hpp"

TranslatedQComboBox::TranslatedQComboBox()
{
}

void TranslatedQComboBox::addItem(const std::string &keyString,
                                  const std::vector<std::string> &parameters)
{
    m_items.push_back({keyString, parameters});
    QComboBox::addItem(QString());
    reloadStrings();
}

void TranslatedQComboBox::reloadStrings()
{
    for (size_t i = 0u; i < m_items.size(); i++)
    {
        setItemText(i, StringLibrary::getTranslatedString(m_items[i].keyString, m_items[i].parameters));
    }
}