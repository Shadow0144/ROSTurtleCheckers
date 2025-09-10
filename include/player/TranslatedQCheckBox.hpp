#pragma once

#include <QCheckBox>

#include <string>
#include <vector>

class TranslatedQCheckBox : public QCheckBox
{
public:
    TranslatedQCheckBox();
    TranslatedQCheckBox(const std::string &keyString,
                        const std::vector<std::string> &parameters = {});

    void setText(const std::string &keyString,
                 const std::vector<std::string> &parameters = {});

    void reloadStrings();

private:
    std::string m_keyString;
    std::vector<std::string> m_parameters;
};