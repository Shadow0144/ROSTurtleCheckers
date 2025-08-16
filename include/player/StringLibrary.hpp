#pragma once

#include <QString>

#include <string>
#include <vector>
#include <unordered_map>

#include "shared/CheckersConsts.hpp"

class StringLibrary
{
public:
    static QString getTranslatedString(const std::string &stringTag,
                                       const std::vector<std::string> &parameters = {});

private:
    static void createLibraryInstance();

    void loadLibrary(std::ifstream &file);

    std::unordered_map<Language, std::unordered_map<std::string, std::string>> m_translatedStringsByLanguage;
};