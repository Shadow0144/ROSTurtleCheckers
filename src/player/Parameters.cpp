#include "player/Parameters.hpp"

#include <string>
#include <memory>

#include "shared/CheckersConsts.hpp"

static std::unique_ptr<Parameters> s_parametersInstance;

void Parameters::createParametersInstance()
{
    s_parametersInstance = std::make_unique<Parameters>();
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