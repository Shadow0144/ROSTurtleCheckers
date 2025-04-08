#include "game_master/TurtlePieceFactory.hpp"

#include "shared/CheckersConsts.hpp"

std::vector<TurtlePiecePtr> TurtlePieceFactory::createTurtlePieces(
    size_t numberOfPiecesPerPlayer,
    const std::vector<TilePtr> &tiles)
{
    std::vector<TurtlePiecePtr> turtlePieces;

    // Black
    for (size_t i = 0u; i < numberOfPiecesPerPlayer; i++)
    {
        std::string name = "Black" + std::to_string(i + 1);
        auto blackTurtle = std::make_shared<TurtlePiece>(name, TurtlePieceColor::Black);
        turtlePieces.push_back(blackTurtle);
        tiles[i + BLACK_OFFSET]->setTurtlePiece(blackTurtle);
    }

    // Red
    for (size_t i = 0u; i < numberOfPiecesPerPlayer; i++)
    {
        std::string name = "Red" + std::to_string(i + 1);
        auto redTurtle = std::make_shared<TurtlePiece>(name, TurtlePieceColor::Red);
        turtlePieces.push_back(redTurtle);
        tiles[i]->setTurtlePiece(redTurtle);
    }

    return turtlePieces;
}