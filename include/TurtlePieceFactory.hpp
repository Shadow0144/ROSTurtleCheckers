#pragma once

#include "Tile.hpp"
#include "TurtlePiece.hpp"

#include <vector>

#include "CheckersConsts.hpp"

class TurtlePieceFactory
{
public:
    static std::vector<TurtlePiecePtr> createTurtlePieces(
        size_t numberOfPiecesPerPlayer,
        const std::vector<TilePtr> &tiles);
};