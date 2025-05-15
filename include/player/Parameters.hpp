#pragma once

#include <string>

#include "shared/CheckersConsts.hpp"

class Parameters
{
public:
    static void setPlayerName(const std::string &playerName);
    static const std::string &getPlayerName();

    static void setPlayerColor(TurtlePieceColor playerColor);
    static TurtlePieceColor getPlayerColor();
    
    static void setLobbyName(const std::string &lobbyName);
    static const std::string &getLobbyName();

    static void setLobbyId(const std::string &lobbyId);
    static const std::string &getLobbyId();

private:
    static void createParametersInstance();

    std::string m_playerName;
    TurtlePieceColor m_playerColor;
    std::string m_lobbyName;
    std::string m_lobbyId;
};