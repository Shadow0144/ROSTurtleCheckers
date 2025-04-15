#pragma once

#include <QMainWindow>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "shared/CheckersConsts.hpp"
#include "player/CheckersMainMenuFrame.hpp"
#include "player/CheckersGameFrame.hpp"

class CheckersPlayerNode;

class CheckersPlayerWindow : public QMainWindow, public std::enable_shared_from_this<CheckersPlayerWindow>
{
    Q_OBJECT
public:
    CheckersPlayerWindow(const std::weak_ptr<CheckersPlayerNode> &playerNode);

    void setConnectedToServer(bool connected);

    void createLobby(const std::string &playerName,
                     const std::string &lobbyName,
                     TurtlePieceColor playerColor);
    void joinLobby(const std::string &playerName,
                   const std::string &lobbyName,
                   const std::string &lobbyId,
                   TurtlePieceColor playerColor);
    void getLobbyList();
    void updateLobbyList(const std::vector<std::string> &lobbyNames,
                         const std::vector<std::string> &lobbyIds,
                         const std::vector<std::string> &blackPlayerNames,
                         const std::vector<std::string> &redPlayerNames);
    void leaveLobby();

    void setReady(bool ready);

    void connectedToLobby(const std::string &lobbyName,
                          const std::string &lobbyId,
                          const std::string &blackPlayerName,
                          const std::string &redPlayerName,
                          bool blackPlayerReady,
                          bool redPlayerReady);

    void playerJoinedLobby(const std::string &playerName, TurtlePieceColor playerColor);
    void playerLeftLobby(const std::string &playerName);
    void setPlayerReady(const std::string &playerName, bool ready);

    void requestedPieceMoveAccepted(bool moveAccepted);
    void requestedReachableTiles(const std::vector<size_t> &reachableTileIndices);
    void declaredWinner(Winner winner);
    void gameStarted(GameState gameState,
                     const std::string &lobbyName,
                     const std::string &lobbyId,
                     const std::string &playerName,
                     TurtlePieceColor playerColor,
                     const std::vector<size_t> &movableTileIndices);
    void updatedBoard(size_t sourceTileIndex, size_t destinationTileIndex, GameState gameState,
                      int slainPieceTileIndex, bool kingPiece, const std::vector<size_t> &movableTileIndices);

    void requestPieceMove(size_t sourceTileIndex, size_t destinationTileIndex);
    void requestReachableTiles(size_t selectedPieceTileIndex);

    void offerDraw();
    void forfit();

    void returnToMainMenu(const std::string &playerName);

protected:
    void closeEvent(QCloseEvent *event) override;

private:
    enum class WindowState
    {
        MainMenu,
        InGame
    };
    WindowState m_windowState;

    std::weak_ptr<CheckersPlayerNode> m_playerNode;

    bool m_connectedToServer;

    CheckersMainMenuFrame *m_checkersMainMenuFrame;
    CheckersGameFrame *m_checkersGameFrame;
};

typedef std::unique_ptr<CheckersPlayerWindow> CheckersPlayerWindowUniPtr;
typedef std::weak_ptr<CheckersPlayerWindow> CheckersPlayerWindowWkPtr;