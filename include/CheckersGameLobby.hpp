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
        const std::string &lobbyName);

    std::string getLobbyName() const;

    bool playerSlotAvailable() const;
    TurtlePieceColor addPlayer(const std::string &playerName);

    Winner getWinner() const;

    void setIsBlackTurn(bool isBlackTurn);
    bool getIsBlackTurn() const;
    void togglePlayerTurn();

    std::vector<uint64_t> requestReachableTiles(const std::string &requestedPieceName) const;
    bool requestPieceMove(const std::string &requestedPieceName, int sourceTileIndex, int destinationTileIndex);
    int getJumpedPieceTileIndex(int sourceTileIndex, int destinationTileIndex) const; // Returns -1 when nothing was jumped
    bool wasPieceKinged(const std::string &pieceName, int destinationTileIndex) const;
    void slayTurtleAtTileIndex(int tileIndex);

private:
    TurtlePieceColor getColorFromPieceName(const std::string &pieceName) const;
    bool isPieceValidForTurn(const std::string &requestedPieceName) const;

    std::string m_lobbyName;
    std::string m_blackPlayerName;
    std::string m_redPlayerName;
    bool m_isBlackTurn;

    size_t m_blackPiecesRemaining;
    size_t m_redPiecesRemaining;

    bool m_isNextMoveBlack;                           // True if it is black's turn, false for red
    std::vector<uint32_t> m_validPiecesPerPlayer_[2]; // Black and red
    std::unordered_map<uint32_t, std::vector<uint32_t>> m_validMovesPerPiece;
    std::vector<TilePtr> m_tiles;
    Winner m_winner;
};

typedef std::shared_ptr<CheckersGameLobby> CheckersGameLobbyPtr;