#pragma once

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "CheckersConsts.hpp"
#include "Tile.hpp"
#include "TurtlePiece.hpp"

class CheckersGameLobby
{
public:
    CheckersGameLobby(const std::string &lobbyName);

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
    bool canJumpAgainFromTileIndex(int tileIndex);
    void addTileToJumpedTileIndices(int tileIndex);
    void slayTurtlesAtJumpedTileIndices();

    void checkPlayersCanMove(std::vector<size_t> &movableTileIndices);

private:
    TurtlePieceColor getColorFromPieceName(const std::string &pieceName) const;
    bool isPieceValidForTurn(const std::string &requestedPieceName) const;

    void slayTurtleAtTileIndex(int tileIndex);

    std::string m_lobbyName;
    std::string m_blackPlayerName;
    std::string m_redPlayerName;
    bool m_isBlackTurn;

    size_t m_blackPiecesRemaining;
    size_t m_redPiecesRemaining;

    std::vector<uint32_t> m_validPiecesPerPlayer_[2]; // Black and red
    std::unordered_map<uint32_t, std::vector<uint32_t>> m_validMovesPerPiece;
    std::vector<TilePtr> m_tiles;

    std::vector<size_t> m_jumpedPieceTileIndices; // Slain turtles are only removed at the end of a turn (i.e. not during an extended jumping session)

    Winner m_winner;

    // Game parameters
    bool m_mustJump = false; // If jumps available, a player must choose one over a regular move
};

typedef std::shared_ptr<CheckersGameLobby> CheckersGameLobbyPtr;