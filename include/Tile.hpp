#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "CheckersConsts.hpp"

#include "TurtlePiece.hpp"

class Tile
{
public:
    Tile(int row, int col);

    int getRow() const;
    int getCol() const;
    
    // Tile statuses
    bool getIsTileReachable() const;
    void setIsTileReachable(bool isReachable);
    bool toggleIsTileReachable();

    bool getIsTileHighlighted() const;
    void setIsTileHighlighted(bool isHighlighted);
    bool toggleIsTileHighlighted();

    bool getIsTileSelected() const;
    void setIsTileSelected(bool isSelected);
    bool toggleIsTileSelected();

    bool getIsTileLastMovedFrom() const;
    void setIsTileLastMovedFrom(bool isLastMovedFrom);
    bool toggleIsTileLastMovedFrom();

    bool getIsTileLastMovedTo() const;
    void setIsTileLastMovedTo(bool isLastMovedTo);
    bool toggleIsTileLastMovedTo();

    // Turtle piece
    virtual void setTurtlePiece(const TurtlePiecePtr &turtle);
    const TurtlePiecePtr &getTurtlePiece() const;
    TurtlePieceColor getTurtlePieceColor() const;
    std::string getTurtlePieceName() const;
    bool containsPiece(TurtlePieceColor color) const;
    void clearTurtlePiece();

    bool getIsTurtlePieceHighlighted() const;
    void setIsTurtlePieceHighlighted(bool isHighlighted);
    bool toggleIsTurtlePieceHighlighted();

    bool getIsTurtlePieceSelected() const;
    void setIsTurtlePieceSelected(bool isSelected);
    bool toggleIsTurtlePieceSelected();

    bool getIsTurtlePieceKinged() const;
    void setIsTurtlePieceKinged(bool isKinged);
    bool toggleIsTurtlePieceKinged();

    bool getIsTurtlePieceDead() const;
    void setIsTurtlePieceDead(bool isDead);
    bool toggleIsTurtlePieceDead();

    void moveTurtlePiece(const std::shared_ptr<Tile> &destinationTile);

    std::vector<uint64_t> getCurrentlyReachableTiles(const std::vector<std::shared_ptr<Tile>> &tiles) const; // Get the list of tiles the piece currently on this tile can reach (including by jumping)

protected:
    bool canJumpPiece(TurtlePieceColor otherPieceColor) const;
    void checkTilesAbove(const std::vector<std::shared_ptr<Tile>> &tiles, std::vector<uint64_t> &reachableTiles) const;
    void checkTilesBelow(const std::vector<std::shared_ptr<Tile>> &tiles, std::vector<uint64_t> &reachableTiles) const;

    int m_row;
    int m_col;

    bool m_isReachable;
    bool m_isHighlighted;
    bool m_isSelected;
    bool m_isLastMovedFrom;
    bool m_isLastMovedTo;

    TurtlePiecePtr m_containedTurtle;
};

typedef std::shared_ptr<Tile> TilePtr;