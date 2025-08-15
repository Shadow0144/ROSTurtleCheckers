#pragma once

#include <string>

#include "shared/CheckersConsts.hpp"

class Parameters
{
public:
    static void setPlayerName(const std::string &playerName);
    static const std::string &getPlayerName();

    static void setOpponentName(const std::string &opponentName);
    static const std::string &getOpponentName();

    static void setPlayerColor(TurtlePieceColor playerColor);
    static TurtlePieceColor getPlayerColor();

    static void setOpponentColor(TurtlePieceColor OpponentColor);
    static TurtlePieceColor getOpponentColor();
    
    static void setLobbyName(const std::string &lobbyName);
    static const std::string &getLobbyName();

    static void setLobbyId(const std::string &lobbyId);
    static const std::string &getLobbyId();

    static void setLanguage(Language language);
    static Language getLanguage();

private:
    static void createParametersInstance();

    std::string m_playerName;
    std::string m_opponentName;
    TurtlePieceColor m_playerColor;
    TurtlePieceColor m_opponentColor;
    std::string m_lobbyName;
    std::string m_lobbyId;

    Language m_language;
};