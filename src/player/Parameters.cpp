#include "player/Parameters.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp" // For getting the settings directory

#include <string>
#include <memory>
#include <filesystem>
#include <fstream>

#include "shared/CheckersConsts.hpp"
#include "shared/TurtleLogger.hpp"

const std::string settingsFile = "settings.config";

static std::unique_ptr<Parameters> s_parametersInstance;

void Parameters::createParametersInstance()
{
    s_parametersInstance = std::make_unique<Parameters>();

    // Get the settings from the file
    try
    {
        std::string fileName = ament_index_cpp::get_package_share_directory("turtle_checkers") + "/" + settingsFile;
        if (std::filesystem::exists(fileName))
        {
            std::ifstream file(fileName);
            if (file)
            {
                std::string language;
                file >> language;
                auto languageEnum = std::stoi(language);
                switch (languageEnum)
                {
                case 0:
                {
                    Parameters::setLanguage(Language::English);
                }
                break;
                case 1:
                {
                    Parameters::setLanguage(Language::Japanese);
                }
                break;
                case 2:
                {
                    Parameters::setLanguage(Language::German);
                }
                break;
                default:
                {
                    Parameters::setLanguage(Language::English);
                }
                break;
                }
            }
            else
            {
                TurtleLogger::logError("Error opening settings file");
            }
        }
        else
        {
            std::ofstream file(fileName);
            file << "0"; // English
            Parameters::setLanguage(Language::English);
        }
    }
    catch (const std::exception &e)
    {
        TurtleLogger::logError(std::string("Settings file error: ") + e.what());
    }
}

void Parameters::setPlayerName(const std::string &playerName)
{
    if (!s_parametersInstance)
    {
        createParametersInstance();
    }

    s_parametersInstance->m_playerName = playerName;
}

const std::string &Parameters::getPlayerName()
{
    if (!s_parametersInstance)
    {
        createParametersInstance();
    }

    return s_parametersInstance->m_playerName;
}

void Parameters::setOpponentName(const std::string &opponentName)
{
    if (!s_parametersInstance)
    {
        createParametersInstance();
    }

    s_parametersInstance->m_opponentName = opponentName;
}

const std::string &Parameters::getOpponentName()
{
    if (!s_parametersInstance)
    {
        createParametersInstance();
    }

    return s_parametersInstance->m_opponentName;
}

void Parameters::setPlayerColor(TurtlePieceColor playerColor)
{
    if (!s_parametersInstance)
    {
        createParametersInstance();
    }

    s_parametersInstance->m_playerColor = playerColor;
}

TurtlePieceColor Parameters::getPlayerColor()
{
    if (!s_parametersInstance)
    {
        createParametersInstance();
    }

    return s_parametersInstance->m_playerColor;
}

void Parameters::setOpponentColor(TurtlePieceColor opponentColor)
{
    if (!s_parametersInstance)
    {
        createParametersInstance();
    }

    s_parametersInstance->m_opponentColor = opponentColor;
}

TurtlePieceColor Parameters::getOpponentColor()
{
    if (!s_parametersInstance)
    {
        createParametersInstance();
    }

    return s_parametersInstance->m_opponentColor;
}

void Parameters::setLobbyName(const std::string &lobbyName)
{
    if (!s_parametersInstance)
    {
        createParametersInstance();
    }

    s_parametersInstance->m_lobbyName = lobbyName;
}

const std::string &Parameters::getLobbyName()
{
    if (!s_parametersInstance)
    {
        createParametersInstance();
    }

    return s_parametersInstance->m_lobbyName;
}

void Parameters::setLobbyId(const std::string &lobbyId)
{
    if (!s_parametersInstance)
    {
        createParametersInstance();
    }

    s_parametersInstance->m_lobbyId = lobbyId;
}

const std::string &Parameters::getLobbyId()
{
    if (!s_parametersInstance)
    {
        createParametersInstance();
    }

    return s_parametersInstance->m_lobbyId;
}

void Parameters::setLanguage(Language language)
{
    if (!s_parametersInstance)
    {
        createParametersInstance();
    }

    s_parametersInstance->m_language = language;

    // Persist the change
    try
    {
        std::ofstream file(ament_index_cpp::get_package_share_directory("turtle_checkers") + "/" + settingsFile, std::ios::trunc);
        if (file)
        {
            file << std::to_string(static_cast<int>(s_parametersInstance->m_language));
        }
        else
        {
            TurtleLogger::logError("Error opening settings file");
        }
    }
    catch (const std::exception &e)
    {
        TurtleLogger::logError(std::string("Settings file error: ") + e.what());
    }

    // Update the UI
    if (s_parametersInstance->m_languageChangedCallback)
    {
        s_parametersInstance->m_languageChangedCallback();
    }
}

Language Parameters::getLanguage()
{
    if (!s_parametersInstance)
    {
        createParametersInstance();
    }

    return s_parametersInstance->m_language;
}

void Parameters::setLanguageChangedCallback(const std::function<void()> &languageChangedCallback)
{
    if (!s_parametersInstance)
    {
        createParametersInstance();
    }

    s_parametersInstance->m_languageChangedCallback = languageChangedCallback;
}