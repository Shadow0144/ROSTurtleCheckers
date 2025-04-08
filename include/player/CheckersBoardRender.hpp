#pragma once

#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QFrame> // NO LINT
#include <QImage>
#include <QPainter>
#include <QPaintEvent>
#include <QTimer>
#include <QVector>
#include <QMouseEvent>
#endif

#include <string>
#include <vector>

#include "shared/CheckersConsts.hpp"
#include "player/TileRender.hpp"
#include "player/TurtlePieceRender.hpp"
#include "player/TurtleGraveyard.hpp"
#include "player/HUD.hpp"

class CheckersBoardRender
{
public:
    CheckersBoardRender();

    void createBoard(TurtlePieceColor playerColor);

    void clearSelections();
    void clearMovedTiles();
    void setMovablePieces(const std::vector<size_t> &movablePieceTileIndices);
    void setReachableTiles(const std::vector<size_t> &reachableTileIndices);
    void moveTurtlePiece(size_t sourceTileIndex, size_t destinationTileIndex);

    void slayTurtle(size_t slainPieceTileIndex);
    void moveTurtlePiecesToGraveyard(
        TurtleGraveyardPtr &blackPlayerGraveyard, TurtleGraveyardPtr &redPlayerGraveyard);

    void kingPiece(size_t kingPieceTileIndex);

    bool getIsMoveSelected();
    int getSelectedPieceTileIndex();
    int getSourceTileIndex();
    int getDestinationTileIndex();

    size_t getBlackTurtlesRemaining();
    size_t getRedTurtlesRemaining();
    
	void handleMouseMove(QMouseEvent *event);
	void handleMouseClick(QMouseEvent *event);

    void paint(QPainter &painter);

private:
    std::vector<TileRenderPtr> m_tileRenders;
    std::vector<TurtlePieceRenderPtr> m_turtlePieceRenders;

    std::vector<size_t> m_tileIndicesOfSlainTurtles; // Indices of tiles containing turtles slain during a multijump

    size_t m_blackTurtlesRemaining;
    size_t m_redTurtlesRemaining;

	int m_selectedPieceTileIndex;
	int m_sourceTileIndex;
	int m_destinationTileIndex;
	bool m_moveSelected;

    int m_highlightedTileIndex = -1; // No tile is highlighted
};

typedef std::shared_ptr<CheckersBoardRender> CheckersBoardRenderPtr;