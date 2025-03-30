#pragma once

#include "Tile.hpp"

#include <vector>

#include "CheckersConsts.hpp"

class TileFactory
{
public:
    static std::vector<TilePtr> createTiles(size_t numRows, size_t numCols);
};