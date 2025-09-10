#pragma once

#include <QPushButton>

#include <string>
#include <vector>

class TranslatedQPushButton : public QPushButton
{
public:
    TranslatedQPushButton();
    TranslatedQPushButton(const std::string &keyString,
                          const std::vector<std::string> &parameters = {});

    void setText(const std::string &keyString,
                 const std::vector<std::string> &parameters = {});

    void reloadStrings();

private:
    std::string m_keyString;
    std::vector<std::string> m_parameters;
};