#pragma once

#include <vector>

#include "CheckersConsts.hpp"
#include "TurtlePieceRender.hpp"
#include "TileRender.hpp"

class TurtlePieceRenderFactory
{
public:
    static std::vector<TurtlePieceRenderPtr> createTurtlePieceRenders(
        size_t numberOfPiecesPerPlayer,
        const std::vector<TileRenderPtr> &tileRenders,
        TurtlePieceColor playerColor);
};