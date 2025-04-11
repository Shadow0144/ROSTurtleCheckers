#include "player/CheckersPlayerWindow.hpp"

#include <QMainWindow>

#include "shared/CheckersConsts.hpp"
#include "player/CheckersPlayerNode.hpp"
#include "player/CheckersMainMenuFrame.hpp"
#include "player/CheckersGameFrame.hpp"

CheckersPlayerWindow::CheckersPlayerWindow(const CheckersPlayerNodeWkPtr &playerNode)
    : QMainWindow()
{
    m_playerNode = playerNode;

    m_windowState = WindowState::MainMenu;

    setFixedSize(WINDOW_WIDTH, WINDOW_HEIGHT);
    setWindowTitle("TurtleCheckers");

    setMouseTracking(true);

    m_checkersMainMenuFrame = std::make_unique<CheckersMainMenuFrame>(this);
    setCentralWidget(m_checkersMainMenuFrame.get());
}

void CheckersPlayerWindow::closeEvent(QCloseEvent *event)
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->shutdown();
    }

    QMainWindow::closeEvent(event);
}

void CheckersPlayerWindow::setConnectedToServer(bool connected)
{
    m_connectedToServer = connected;
    if (m_checkersMainMenuFrame)
    {
        m_checkersMainMenuFrame->setConnectedToServer(connected);
    }
}

void CheckersPlayerWindow::createLobby(const std::string &playerName,
                                       const std::string &lobbyName,
                                       TurtlePieceColor playerColor)
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->createLobby(playerName, lobbyName, playerColor);
    }
}

void CheckersPlayerWindow::joinLobby(const std::string &playerName,
                                     const std::string &lobbyName,
                                     const std::string &lobbyId,
                                     TurtlePieceColor playerColor)
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->joinLobby(playerName, lobbyName, lobbyId, playerColor);
    }
}

void CheckersPlayerWindow::getLobbyList()
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->getLobbyList();
    }
}

void CheckersPlayerWindow::updateLobbyList(const std::vector<std::string> &lobbyNames,
                                           const std::vector<std::string> &lobbyIds,
                                           const std::vector<std::string> &blackPlayerNames,
                                           const std::vector<std::string> &redPlayerNames)
{
    m_checkersMainMenuFrame->displayLobbyList(lobbyNames, lobbyIds, blackPlayerNames, redPlayerNames);
}

void CheckersPlayerWindow::leaveLobby()
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->leaveLobby();
    }
}

void CheckersPlayerWindow::setReady(bool ready)
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->setReady(ready);
    }
}

void CheckersPlayerWindow::connectedToLobby(const std::string &lobbyName,
                                            const std::string &lobbyId,
                                            const std::string &blackPlayerName,
                                            const std::string &redPlayerName,
                                            bool blackPlayerReady,
                                            bool redPlayerReady)
{
    m_checkersMainMenuFrame->connectedToLobby(lobbyName,
                                              lobbyId,
                                              blackPlayerName,
                                              redPlayerName,
                                              blackPlayerReady,
                                              redPlayerReady);
}

void CheckersPlayerWindow::playerJoinedLobby(const std::string &playerName, TurtlePieceColor playerColor)
{
    m_checkersMainMenuFrame->playerJoinedLobby(playerName, playerColor);
}

void CheckersPlayerWindow::playerLeftLobby(const std::string &playerName)
{
    m_checkersMainMenuFrame->playerLeftLobby(playerName);
}

void CheckersPlayerWindow::setPlayerReady(const std::string &playerName, bool ready)
{
    m_checkersMainMenuFrame->setPlayerReady(playerName, ready);
}

void CheckersPlayerWindow::requestedPieceMoveAccepted(bool moveAccepted)
{
    m_checkersGameFrame->requestedPieceMoveAccepted(moveAccepted);
}

void CheckersPlayerWindow::requestedReachableTiles(const std::vector<size_t> &reachableTileIndices)
{
    m_checkersGameFrame->requestedReachableTiles(reachableTileIndices);
}

void CheckersPlayerWindow::declaredWinner(Winner winner)
{
    m_checkersGameFrame->declaredWinner(winner);
}

void CheckersPlayerWindow::gameStarted(GameState gameState,
                                       const std::string &playerName,
                                       const std::string &lobbyName,
                                       const std::string &lobbyId,
                                       TurtlePieceColor playerColor,
                                       const std::vector<size_t> &movableTileIndices)
{
    m_checkersGameFrame = std::make_unique<CheckersGameFrame>(this, playerName);
    setCentralWidget(m_checkersGameFrame.get());

    m_checkersGameFrame->connectedToGame(lobbyName, lobbyId, playerColor);
    m_checkersGameFrame->gameStarted(gameState, movableTileIndices);

    update();
}

void CheckersPlayerWindow::updatedBoard(size_t sourceTileIndex, size_t destinationTileIndex,
                                        GameState gameState, int slainPieceTileIndex, bool kingPiece,
                                        const std::vector<size_t> &movableTileIndices)
{
    m_checkersGameFrame->updatedBoard(sourceTileIndex, destinationTileIndex, gameState,
                                      slainPieceTileIndex, kingPiece, movableTileIndices);
}

void CheckersPlayerWindow::requestPieceMove(size_t sourceTileIndex, size_t destinationTileIndex)
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->requestPieceMove(sourceTileIndex, destinationTileIndex);
    }
}

void CheckersPlayerWindow::requestReachableTiles(size_t selectedPieceTileIndex)
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->requestReachableTiles(selectedPieceTileIndex);
    }
}

void CheckersPlayerWindow::update()
{
    m_checkersGameFrame->update();
}