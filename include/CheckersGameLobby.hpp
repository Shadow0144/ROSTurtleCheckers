#pragma once

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "CheckersConsts.hpp"
#include "Tile.hpp"

class CheckersGameLobby
{
public:
    CheckersGameLobby(
        const std::string &lobbyName,
        const std::string &blackPlayerName,
        const std::string &redPlayerName);

    std::string getLobbyName();

    Winner getWinner();

    std::vector<uint64_t> requestReachableTiles(const std::string &requestedPieceName);

private:
    std::string m_lobbyName;
    std::string m_blackPlayerName;
    std::string m_redPlayerName;

    bool m_isNextMoveBlack;                           // True if it is black's turn, false for red
    std::vector<uint32_t> m_validPiecesPerPlayer_[2]; // Black and red
    std::unordered_map<uint32_t, std::vector<uint32_t>> m_validMovesPerPiece;
    std::vector<TilePtr> m_tiles;
    Winner m_winner;
};

typedef std::shared_ptr<CheckersGameLobby> CheckersGameLobbyPtr;