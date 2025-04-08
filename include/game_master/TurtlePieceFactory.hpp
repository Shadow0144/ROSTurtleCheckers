#pragma once

#include <vector>

#include "shared/CheckersConsts.hpp"
#include "shared/Tile.hpp"
#include "shared/TurtlePiece.hpp"

class TurtlePieceFactory
{
public:
    static std::vector<TurtlePiecePtr> createTurtlePieces(
        size_t numberOfPiecesPerPlayer,
        const std::vector<TilePtr> &tiles);
};