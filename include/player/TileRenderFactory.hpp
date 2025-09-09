#pragma once

#include <memory>
#include <vector>

#include "shared/CheckersConsts.hpp"
#include "player/TileRender.hpp"

class TileRenderFactory
{
public:
    static std::vector<TileRenderPtr> createTileRenders(
        size_t numRows, size_t numCols, TurtlePieceColor playerColor);
};