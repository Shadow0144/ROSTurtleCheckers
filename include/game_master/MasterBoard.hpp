#pragma once

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "shared/CheckersConsts.hpp"
#include "shared/Tile.hpp"
#include "shared/TurtlePiece.hpp"

class MasterBoard
{
public:
    MasterBoard();

    Winner getWinner() const;

    bool getMustJump() const;
    void setMustJump(bool mustJump);

    std::string getPieceNameAtTileIndex(int tileIndex) const;
    TurtlePieceColor getPieceColorAtTileIndex(int tileIndex) const;
    std::vector<uint64_t> requestReachableTiles(int tileIndex) const;
    std::vector<uint64_t> requestJumpableTiles(int tileIndex) const;
    bool requestPieceMove(int sourceTileIndex, int destinationTileIndex);
    int getJumpedPieceTileIndex(int sourceTileIndex, int destinationTileIndex) const; // Returns -1 when nothing was jumped
    bool wasPieceKinged(int tileIndex) const;
    bool canJumpAgainFromTileIndex(int tileIndex);
    void addTileToJumpedTileIndices(int tileIndex);
    void slayTurtlesAtJumpedTileIndices();

    void checkPlayersCanMove(bool isBlackTurn, std::vector<size_t> &movableTileIndices);

private:
    TurtlePieceColor getColorFromPieceName(const std::string &pieceName) const;

    void slayTurtleAtTileIndex(int tileIndex);

    size_t m_blackPiecesRemaining;
    size_t m_redPiecesRemaining;

    std::vector<TilePtr> m_tiles;

    std::vector<size_t> m_jumpedPieceTileIndices; // Slain turtles are only removed at the end of a turn (i.e. not during an extended jumping session)

    Winner m_winner;

    // Game parameters
    bool m_mustJump = false; // If jumps available, a player must choose one over a regular move

    friend struct std::hash<std::shared_ptr<MasterBoard>>;
};

typedef std::shared_ptr<MasterBoard> MasterBoardPtr;

template <>
struct std::hash<MasterBoardPtr>
{
    size_t operator()(const MasterBoardPtr &masterBoardPtr) const noexcept;
};