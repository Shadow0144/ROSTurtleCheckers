#pragma once

#include <QMainWindow>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <QStackedLayout>

#include "shared/CheckersConsts.hpp"

#include "player/frame/CreateAccountFrame.hpp"
#include "player/frame/CreateLobbyFrame.hpp"
#include "player/frame/GameFrame.hpp"
#include "player/frame/InLobbyFrame.hpp"
#include "player/frame/LobbyListFrame.hpp"
#include "player/frame/LobbyPasswordFrame.hpp"
#include "player/frame/LogInAccountFrame.hpp"
#include "player/frame/MainMenuFrame.hpp"
#include "player/frame/TitleFrame.hpp"

class CheckersPlayerNode;

class CheckersPlayerWindow : public QMainWindow, public std::enable_shared_from_this<CheckersPlayerWindow>
{
    Q_OBJECT
public:
    CheckersPlayerWindow(const std::weak_ptr<CheckersPlayerNode> &playerNode);

    void setConnectedToServer(bool connected);

    void moveToTitleFrame();
    void moveToCreateAccountFrame();
    void moveToLogInAccountFrame();
    void moveToMainMenuFrame();
    void moveToCreateLobbyFrame();
    void moveToLobbyListFrame();
    void moveToLobbyPasswordFrame();
    void moveToInLobbyFrame();
    void moveToGameFrame();

    void createAccount(const std::string &playerName, const std::string &playerPassword);
    void logInAccount(const std::string &playerName, const std::string &playerPassword);
    void logOutAccount();

    void createLobby(const std::string &lobbyPassword);
    void joinLobby(const std::string &lobbyPassword);
    void getLobbyList();
    void updateLobbyList(const std::vector<std::string> &lobbyNames,
                         const std::vector<std::string> &lobbyIds,
                         const std::vector<bool> &hasPasswords,
                         const std::vector<std::string> &blackPlayerNames,
                         const std::vector<std::string> &redPlayerNames);
    void setPasswordIncorrect();
    void leaveLobby();

    void setReady(bool ready);

    void loggedIn(const std::string &playerName);
    void failedLogIn(const std::string &errorMessage);
    void accountCreated(const std::string &playerName);
    void failedCreate(const std::string &errorMessage);

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
                     const std::vector<size_t> &movableTileIndices);
    void updatedBoard(size_t sourceTileIndex, size_t destinationTileIndex, GameState gameState,
                      int slainPieceTileIndex, bool kingPiece, const std::vector<size_t> &movableTileIndices);

    void requestPieceMove(size_t sourceTileIndex, size_t destinationTileIndex);
    void requestReachableTiles(size_t selectedPieceTileIndex);

    void offerDraw();
    void declineDraw();
    void forfit();

    void drawDeclined();
    void drawOffered();

protected:
    void closeEvent(QCloseEvent *event) override;

private:
    enum class WindowState
    {
        CreateAccount,
        CreateLobby,
        Game,
        LobbyList,
        LobbyPassword,
        LogInAccount,
        MainMenu,
        Title
    };
    WindowState m_windowState;

    QStackedLayout *m_windowLayout;
    static constexpr int CREATE_ACCOUNT_INDEX = 0;
    static constexpr int CREATE_LOBBY_INDEX = 1;
    static constexpr int GAME_INDEX = 2;
    static constexpr int IN_LOBBY_INDEX = 3;
    static constexpr int LOBBY_LIST_INDEX = 4;
    static constexpr int LOBBY_PASSWORD_INDEX = 5;
    static constexpr int LOG_IN_ACCOUNT_INDEX = 6;
    static constexpr int MAIN_MENU_INDEX = 7;
    static constexpr int TITLE_INDEX = 8;

    std::weak_ptr<CheckersPlayerNode> m_playerNode;

    bool m_connectedToServer;

    CreateAccountFrame *m_createAccountFrame;
    CreateLobbyFrame *m_createLobbyFrame;
    GameFrame *m_gameFrame;
    InLobbyFrame *m_inLobbyFrame;
    LobbyListFrame *m_lobbyListFrame;
    LobbyPasswordFrame *m_lobbyPasswordFrame;
    LogInAccountFrame *m_logInAccountFrame;
    MainMenuFrame *m_mainMenuFrame;
    TitleFrame *m_titleFrame;
};

typedef std::unique_ptr<CheckersPlayerWindow> CheckersPlayerWindowUniPtr;
typedef std::weak_ptr<CheckersPlayerWindow> CheckersPlayerWindowWkPtr;