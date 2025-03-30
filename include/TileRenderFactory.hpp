#pragma once

#include <vector>

#include "CheckersConsts.hpp"
#include "TileRender.hpp"

class TileRenderFactory
{
public:
    static std::vector<TileRenderPtr> createTileRenders(
        size_t numRows, size_t numCols, TurtlePieceColor playerColor);
};