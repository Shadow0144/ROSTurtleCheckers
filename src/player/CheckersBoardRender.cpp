#include "player/CheckersBoardRender.hpp"

#include <QPointF>
#include <QPainter>
#include <QMouseEvent>

#include <cstdlib>
#include <ctime>
#include <functional>
#include <string>
#include <chrono>
#include <vector>
#include <iostream>

#include "shared/CheckersConsts.hpp"
#include "shared/Hasher.hpp"
#include "player/TileRenderFactory.hpp"
#include "player/TurtlePieceRenderFactory.hpp"

CheckersBoardRender::CheckersBoardRender()
{
    m_selectedPieceTileIndex = -1;
    m_sourceTileIndex = -1;
    m_destinationTileIndex = -1;
    m_moveSelected = false;
}

void CheckersBoardRender::createBoard(TurtlePieceColor playerColor)
{
    m_tileRenders = TileRenderFactory::createTileRenders(NUM_PLAYABLE_ROWS, NUM_PLAYABLE_COLS, playerColor);
    m_turtlePieceRenders = TurtlePieceRenderFactory::createTurtlePieceRenders(NUM_PIECES_PER_PLAYER, m_tileRenders, playerColor);

    m_blackTurtlesRemaining = NUM_PIECES_PER_PLAYER;
    m_redTurtlesRemaining = NUM_PIECES_PER_PLAYER;
}

void CheckersBoardRender::clearSelections()
{
    for (auto &tileRender : m_tileRenders)
    {
        tileRender->setIsTileReachable(false);
        tileRender->setIsTileHighlighted(false);
        tileRender->setIsTileSelected(false);
        tileRender->setIsTurtlePieceSelected(false);
    }
    m_selectedPieceTileIndex = -1;
    m_sourceTileIndex = -1;
    m_destinationTileIndex = -1;
    m_moveSelected = false;
}

void CheckersBoardRender::clearMovedTiles()
{
    for (auto &tileRender : m_tileRenders)
    {
        tileRender->setIsTileLastMovedFrom(false);
        tileRender->setIsTileLastMovedTo(false);
        tileRender->setIsTileLastJumpedOver(false);
        tileRender->setIsTurtlePieceMovable(false);
    }
}

void CheckersBoardRender::setMovablePieces(const std::vector<size_t> &movablePieceTileIndices)
{
    for (auto &tileRender : m_tileRenders)
    {
        tileRender->setIsTurtlePieceMovable(false);
    }
    for (auto movableTileIndex : movablePieceTileIndices)
    {
        if (movableTileIndex < NUM_PLAYABLE_TILES)
        {
            m_tileRenders[movableTileIndex]->setIsTurtlePieceMovable(true);
        }
    }
}

void CheckersBoardRender::setReachableTiles(const std::vector<size_t> &reachableTileIndices)
{
    for (auto &tileRender : m_tileRenders)
    {
        tileRender->setIsTileReachable(false);
    }
    for (const auto reachableTileIndex : reachableTileIndices)
    {
        m_tileRenders[reachableTileIndex]->setIsTileReachable(true);
    }
}

void CheckersBoardRender::clearTurtlePieces()
{
    for (auto tileRender : m_tileRenders)
    {
        tileRender->clearTurtlePiece();
    }
}

void CheckersBoardRender::moveTurtlePiece(size_t sourceTileIndex, size_t destinationTileIndex)
{
    if (sourceTileIndex < NUM_PLAYABLE_TILES &&
        destinationTileIndex < NUM_PLAYABLE_TILES)
    {
        m_tileRenders[sourceTileIndex]->moveTurtlePiece(m_tileRenders[destinationTileIndex]);
        m_tileRenders[sourceTileIndex]->setIsTileLastMovedFrom(true);
        m_tileRenders[destinationTileIndex]->setIsTileLastMovedTo(true);
    }
}

void CheckersBoardRender::slayTurtle(size_t slainPieceTileIndex)
{
    if (slainPieceTileIndex < static_cast<int>(NUM_PLAYABLE_TILES))
    {
        m_tileRenders[slainPieceTileIndex]->setIsTileLastJumpedOver(true);
        m_tileRenders[slainPieceTileIndex]->setIsTurtlePieceDead(true);
    }
    m_tileIndicesOfSlainTurtles.push_back(slainPieceTileIndex);
}

void CheckersBoardRender::moveTurtlePiecesToGraveyard(
    TurtleGraveyardPtr &blackPlayerGraveyard, TurtleGraveyardPtr &redPlayerGraveyard)
{
    for (auto tileIndex : m_tileIndicesOfSlainTurtles)
    {
        switch (m_tileRenders[tileIndex]->getTurtlePieceColor())
        {
        case TurtlePieceColor::Black:
        {
            m_tileRenders[tileIndex]->moveTurtlePiece(redPlayerGraveyard);
            m_blackTurtlesRemaining--;
        }
        break;
        case TurtlePieceColor::Red:
        {
            m_tileRenders[tileIndex]->moveTurtlePiece(blackPlayerGraveyard);
            m_redTurtlesRemaining--;
        }
        break;
        case TurtlePieceColor::None:
        {
            // Do nothing
        }
        break;
        }
    }
    m_tileIndicesOfSlainTurtles.clear();
}

void CheckersBoardRender::kingPiece(size_t kingPieceTileIndex)
{
    m_tileRenders[kingPieceTileIndex]->setIsTurtlePieceKinged(true);
}

void CheckersBoardRender::resyncBoard(std::vector<std::string> turtlePieceNamePerTile,
                                      std::vector<uint64_t> turtlePieceColorPerTile,
                                      std::vector<bool> turtlePieceIsKingedPerTile,
                                      TurtleGraveyardPtr &blackPlayerGraveyard,
                                      TurtleGraveyardPtr &redPlayerGraveyard)
{
    const auto tileCount = m_tileRenders.size();
    if (turtlePieceNamePerTile.size() != tileCount ||
        turtlePieceColorPerTile.size() != tileCount ||
        turtlePieceIsKingedPerTile.size() != tileCount)
    {
        std::cerr << "Resync failed: Malformed vectors" << std::endl;
        return;
    }

    clearTurtlePieces();
    redPlayerGraveyard->clear();
    blackPlayerGraveyard->clear();

    // Find a home (or grave) for every turtle piece
    const size_t pieceCount = m_turtlePieceRenders.size();
    for (size_t p = 0u; p < pieceCount; p++)
    {
        bool alive = false;
        for (size_t t = 0u; t < tileCount; t++)
        {
            if (turtlePieceNamePerTile[t] == m_turtlePieceRenders[p]->getName())
            {
                // We've found its home
                m_tileRenders[t]->setTurtlePiece(m_turtlePieceRenders[p]);
                m_tileRenders[t]->setIsTurtlePieceKinged(turtlePieceIsKingedPerTile[t]);
                alive = true;
                break;
            }
        }
        if (!alive)
        {
            // The piece wasn't found on any tile, so it must belong to a graveyard
            switch (m_turtlePieceRenders[p]->getColor())
            {
            case TurtlePieceColor::Black:
            {
                redPlayerGraveyard->addTurtlePiece(m_turtlePieceRenders[p]);
            }
            break;
            case TurtlePieceColor::Red:
            {
                blackPlayerGraveyard->addTurtlePiece(m_turtlePieceRenders[p]);
            }
            break;
            case TurtlePieceColor::None:
            {
                // Do nothing
            }
            break;
            }
        }
    }
}

size_t CheckersBoardRender::getBlackTurtlesRemaining()
{
    return m_blackTurtlesRemaining;
}

size_t CheckersBoardRender::getRedTurtlesRemaining()
{
    return m_redTurtlesRemaining;
}

void CheckersBoardRender::handleMouseMove(QMouseEvent *event)
{
    if (!m_moveSelected)
    {
        for (auto &tileRender : m_tileRenders)
        {
            tileRender->setIsTurtlePieceHighlighted(false);
        }
        for (auto &tileRender : m_tileRenders)
        {
            if (tileRender->getIsTurtlePieceMovable() &&
                tileRender->containsPoint(event->pos()))
            {
                tileRender->setIsTurtlePieceHighlighted(true);
                break;
            }
        }

        if (m_selectedPieceTileIndex > -1)
        {
            for (auto &tileRender : m_tileRenders)
            {
                tileRender->setIsTileHighlighted(false);
            }
            for (auto &tileRender : m_tileRenders)
            {
                if (tileRender->getIsTileReachable() &&
                    tileRender->containsPoint(event->pos()))
                {
                    tileRender->setIsTileHighlighted(true);
                    break;
                }
            }
        }
    }
}

void CheckersBoardRender::handleMouseClick(QMouseEvent *event)
{
    if (m_moveSelected)
    {
        // Do nothing, continue to wait for a response from the game node
    }
    else if (m_selectedPieceTileIndex == -1) // If no piece is selected
    {
        for (auto &tileRender : m_tileRenders)
        {
            tileRender->setIsTurtlePieceSelected(false);
            tileRender->setIsTileReachable(false);
            tileRender->setIsTileHighlighted(false);
            tileRender->setIsTileSelected(false);
        }
        bool selected = false;
        for (int i = 0; i < static_cast<int>(NUM_PLAYABLE_TILES); i++)
        {
            if (m_tileRenders[i]->containsPoint(event->pos()) &&
                m_tileRenders[i]->getIsTurtlePieceMovable())
            {
                if (m_selectedPieceTileIndex != i) // If we've clicked a new piece, select it
                {
                    m_sourceTileIndex = i;
                    m_selectedPieceTileIndex = i;
                    m_tileRenders[m_selectedPieceTileIndex]->setIsTurtlePieceSelected(true);
                    selected = true;
                }
                else // If we've clicked the same piece again, unselect it instead
                {
                    m_selectedPieceTileIndex = -1;
                }
                break; // Only 1 tile can contain the mouse at any time
            }
        }
        if (!selected) // We clicked somewhere which isn't on a valid turtle, so clear the selected piece
        {
            m_selectedPieceTileIndex = -1;
        }
    }
    else // A piece is selected, look for a tile to accept
    {
        bool selected = false;
        for (size_t i = 0u; i < NUM_PLAYABLE_TILES; i++)
        {
            if (m_tileRenders[i]->containsPoint(event->pos()))
            {
                if (m_tileRenders[i]->getIsTileReachable())
                {
                    m_destinationTileIndex = i;
                    m_tileRenders[i]->setIsTileSelected(true);
                    for (auto &tileRender : m_tileRenders)
                    {
                        tileRender->setIsTileReachable(false);
                        tileRender->setIsTileHighlighted(false);
                        tileRender->setIsTurtlePieceMovable(false);
                    }
                    m_moveSelected = true;
                    selected = true;
                }
                break; // Only 1 tile can contain the mouse at any time
            }
        }
        if (!selected) // We clicked somewhere which isn't on a valid turtle, so clear the selected piece
        {
            clearSelections();
        }
    }
}

bool CheckersBoardRender::getIsMoveSelected()
{
    return m_moveSelected;
}

int CheckersBoardRender::getSelectedPieceTileIndex()
{
    return m_selectedPieceTileIndex;
}

int CheckersBoardRender::getSourceTileIndex()
{
    return m_sourceTileIndex;
}

int CheckersBoardRender::getDestinationTileIndex()
{
    return m_destinationTileIndex;
}

void CheckersBoardRender::paint(QPainter &painter)
{
    // Fill the board background in red
    QRgb backgroundColor = qRgb(RED_SQUARES_BG_RGB[0], RED_SQUARES_BG_RGB[1], RED_SQUARES_BG_RGB[2]);
    painter.fillRect(BOARD_LEFT, BOARD_TOP, BOARD_WIDTH, BOARD_HEIGHT, backgroundColor);

    // Draw the black tiles over the red background and any turtles they contain
    for (auto &tileRender : m_tileRenders)
    {
        tileRender->paint(painter);
    }
}

size_t std::hash<CheckersBoardRenderPtr>::operator()(const CheckersBoardRenderPtr &checkersBoardRenderPtr) const noexcept
{
    // This must match the game_master/MasterBoard hash
    size_t combinedHash = 0u;
    hashCombine(combinedHash, std::hash<size_t>{}(checkersBoardRenderPtr->m_blackTurtlesRemaining));
    hashCombine(combinedHash, std::hash<size_t>{}(checkersBoardRenderPtr->m_redTurtlesRemaining));
    hashCombine(combinedHash, std::hash<std::vector<TileRenderPtr>>{}(checkersBoardRenderPtr->m_tileRenders));
    hashCombine(combinedHash, std::hash<std::vector<uint64_t>>{}(checkersBoardRenderPtr->m_tileIndicesOfSlainTurtles));
    return combinedHash;
}