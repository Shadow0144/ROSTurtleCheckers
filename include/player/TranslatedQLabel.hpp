#pragma once

#include <QLabel>

#include <string>
#include <vector>

class TranslatedQLabel : public QLabel
{
public:
    TranslatedQLabel();
    TranslatedQLabel(const std::string &keyString,
                     const std::vector<std::string> &parameters = {});

    void setText(const std::string &keyString,
                 const std::vector<std::string> &parameters = {});

    void reloadStrings();

private:
    std::string m_keyString;
    std::vector<std::string> m_parameters;
};