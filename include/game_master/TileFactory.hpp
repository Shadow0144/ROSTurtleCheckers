#pragma once

#include <vector>

#include "shared/CheckersConsts.hpp"
#include "shared/Tile.hpp"

class TileFactory
{
public:
    static std::vector<TilePtr> createTiles(size_t numRows, size_t numCols);
};