#include "player/StringLibrary.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp" // For getting the string directory

#include <string>
#include <vector>
#include <filesystem>
#include <fstream>
#include <regex>
#include <iostream>

#include "shared/CheckersConsts.hpp"
#include "shared/TurtleLogger.hpp"
#include "player/Parameters.hpp"

const std::string stringsFile = "translations.xml";

static std::unique_ptr<StringLibrary> s_libraryInstance;

bool stringContains(const std::string &str, const std::string &subStr)
{
    return (str.find(subStr) != std::string::npos);
}

void StringLibrary::createLibraryInstance()
{
    s_libraryInstance = std::make_unique<StringLibrary>();

    // Get the translations from the file
    try
    {
        std::ifstream file(ament_index_cpp::get_package_share_directory("turtle_checkers") +
                           std::string("/") + stringsFile);
        if (file)
        {
            if (file.is_open())
            {
                s_libraryInstance->loadLibrary(file);
                file.close();
            }
            else
            {
                TurtleLogger::logError("Unable to translations file");
            }
        }
        else
        {
            TurtleLogger::logError("Error opening translations file");
        }
    }
    catch (const std::exception &e)
    {
        TurtleLogger::logError(std::string("Translations file error: ") + e.what());
    }
}

void StringLibrary::loadLibrary(std::ifstream &file)
{
    m_translatedStringsByLanguage[Language::English] = std::unordered_map<std::string, std::string>{};
    m_translatedStringsByLanguage[Language::Japanese] = std::unordered_map<std::string, std::string>{};
    m_translatedStringsByLanguage[Language::German] = std::unordered_map<std::string, std::string>{};

    std::string line;
    bool inTranslations = false;
    bool inString = false;
    std::string stringKey = "";
    std::string translation = "";
    while (std::getline(file, line))
    {
        if (stringContains(line, "<translations>"))
        {
            inTranslations = true;
        }
        else if (stringContains(line, "</translations>"))
        {
            inTranslations = true;
        }
        else if (inTranslations && stringContains(line, "<string>"))
        {
            inString = true;
        }
        else if (inString && stringContains(line, "</string>"))
        {
            inString = false;
            stringKey = "";
        }
        else if (inString && stringContains(line, "<key>") && stringContains(line, "</key>"))
        {
            auto start = line.find("<key>") + std::string("<key>").length();
            auto length = line.find("</key>") - start;
            stringKey = line.substr(start, length);
        }
        else if (inString && !stringKey.empty())
        {
            if (stringContains(line, "<en>") && stringContains(line, "</en>"))
            {
                auto start = line.find("<en>") + std::string("<en>").length();
                auto length = line.find("</en>") - start;
                translation = line.substr(start, length);
                m_translatedStringsByLanguage[Language::English][stringKey] = translation;
            }
            else if (stringContains(line, "<jp>") && stringContains(line, "</jp>"))
            {
                auto start = line.find("<jp>") + std::string("<jp>").length();
                auto length = line.find("</jp>") - start;
                translation = line.substr(start, length);
                m_translatedStringsByLanguage[Language::Japanese][stringKey] = translation;
            }
            else if (stringContains(line, "<de>") && stringContains(line, "</de>"))
            {
                auto start = line.find("<de>") + std::string("<de>").length();
                auto length = line.find("</de>") - start;
                translation = line.substr(start, length);
                m_translatedStringsByLanguage[Language::German][stringKey] = translation;
            }
        }
    }
}

std::string StringLibrary::getTranslatedString(const std::string &stringTag,
                                               const std::vector<std::string> &parameters)
{
    if (!s_libraryInstance)
    {
        createLibraryInstance();
    }

    std::string returnString = "ERR";

    auto language = Parameters::getLanguage();
    auto &translatedStrings = s_libraryInstance->m_translatedStringsByLanguage[language];
    if (translatedStrings.find(stringTag) != translatedStrings.end())
    {
        try
        {
            returnString = translatedStrings[stringTag];
            // Fill in the parameters
            for (const auto &parameter : parameters)
            {
                returnString.replace(returnString.find("%s"), parameter.length(), parameter);
            }
        }
        catch (const std::exception &)
        {
            TurtleLogger::logError("Translated string given incorrect number of parameters");
        }
    }
    else
    {
        TurtleLogger::logError("Translation of string missing");
    }

    return returnString;
}