#pragma once

#include <QComboBox>

#include <string>
#include <vector>

class TranslatedQComboBox : public QComboBox
{
public:
    TranslatedQComboBox();

    void addItem(const std::string &keyString,
                 const std::vector<std::string> &parameters = {});

    void reloadStrings();

private:
    struct item
    {
        std::string keyString;
        std::vector<std::string> parameters;
    };
    std::vector<item> m_items;
};