#pragma once

#include <QMainWindow>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <QWidget>
#include <QStackedLayout>

#include "shared/CheckersConsts.hpp"

#include "player/frame/ChangeAccountPasswordFrame.hpp"
#include "player/frame/CreateAccountFrame.hpp"
#include "player/frame/CreateLobbyFrame.hpp"
#include "player/frame/GameFrame.hpp"
#include "player/frame/InLobbyFrame.hpp"
#include "player/frame/LobbyListFrame.hpp"
#include "player/frame/LobbyPasswordFrame.hpp"
#include "player/frame/LogInAccountFrame.hpp"
#include "player/frame/MainMenuFrame.hpp"
#include "player/frame/StatisticsFrame.hpp"
#include "player/frame/TitleFrame.hpp"
#include "player/DialogWidget.hpp"

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
    void moveToChangeAccountPasswordFrame();
    void moveToStatisticsFrame();
    void moveToCreateLobbyFrame();
    void moveToLobbyListFrame();
    void moveToLobbyPasswordFrame();
    void moveToInLobbyFrame();
    void moveToGameFrame();

    void createAccount(const std::string &playerName, const std::string &playerPassword);
    void logInAccount(const std::string &playerName, const std::string &playerPassword);
    void logOutAccount();

    void requestStatistics(const std::string &playerName);
    void displayStatistics(const std::string &playerName,
                           const std::vector<std::string> &lobbyNameIds,
                           const std::vector<std::string> &blackPlayerNames,
                           const std::vector<std::string> &redPlayerNames,
                           const std::vector<uint64_t> &winners,
                           uint64_t matchesPlayed,
                           uint64_t matchesWon,
                           uint64_t matchesLost,
                           uint64_t matchesDrawn);

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
    void setTimer(uint64_t timerSeconds);

    void loggedIn(const std::string &playerName);
    void failedLogIn(const std::string &errorMessage);
    void accountCreated(const std::string &playerName);
    void failedCreate(const std::string &errorMessage);

    void changeAccountPassword(const std::string &previousPlayerPassword,
                               const std::string &newPlayerPassword);
    void accountPasswordChanged();
    void failedAccountPasswordChange(const std::string &errorMessage);

    void connectedToLobby(const std::string &lobbyName,
                          const std::string &lobbyId,
                          const std::string &blackPlayerName,
                          const std::string &redPlayerName,
                          TurtlePieceColor lobbyOwnerColor,
                          bool blackPlayerReady,
                          bool redPlayerReady,
                          uint64_t timerSeconds);

    void playerJoinedLobby(const std::string &playerName, TurtlePieceColor playerColor);
    void playerLeftLobby(const std::string &playerName);
    void updateLobbyOwner(const std::string &playerName);
    void kickPlayer(const std::string &playerName);
    void setPlayerReady(const std::string &playerName, bool ready);
    void updateTimer(uint64_t timerSeconds);

    void reportPlayer(const std::string &chatMessages);

    void addChatMessage(const std::string &playerName,
                        TurtlePieceColor playerColor,
                        const std::string &chatMessage,
                        std::chrono::time_point<std::chrono::system_clock> timeStamp);
    void sendChatMessage(const std::string &chatMessage);

    void requestedPieceMoveAccepted(bool moveAccepted);
    void requestedReachableTiles(const std::vector<size_t> &reachableTileIndices);
    void declaredWinner(Winner winner);
    void gameStarted(GameState gameState,
                     const std::vector<size_t> &movableTileIndices,
                     size_t blackTimeRemainSec, size_t redTimeRemainSec);
    void updatedBoard(size_t sourceTileIndex, size_t destinationTileIndex, GameState gameState,
                      int slainPieceTileIndex, bool kingPiece, const std::vector<size_t> &movableTileIndices,
                      size_t blackTimeRemainSec, size_t redTimeRemainSec);
    void resyncBoard(uint64_t blackTimeRemainingSeconds,
                     uint64_t redTimeRemainingSeconds,
                     uint64_t gameState,
                     uint64_t blackPiecesRemaining,
                     uint64_t redPiecesRemaining,
                     std::vector<std::string> turtlePieceNamePerTile,
                     std::vector<uint64_t> turtlePieceColorPerTile,
                     std::vector<bool> turtlePieceIsKingedPerTile);

    void requestPieceMove(size_t sourceTileIndex, size_t destinationTileIndex);
    void requestReachableTiles(size_t selectedPieceTileIndex);

    void offerDraw();
    void declineDraw();
    void forfit();

    void drawDeclined();
    void drawOffered();

    void disconnected();
    void loggedOut();
    void kicked();
    void banned();

    void displayDialog(bool dialogDisplayed, DialogWidget *dialog = nullptr);

    void handleReturnToTitle();
    void handleReturnToLobbyList();

    uint64_t getBoardHash() const;

    void reloadStrings();

protected:
    void closeEvent(QCloseEvent *event) override;

private:
    enum class WindowState
    {
        ChangeAccountPassword,
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

    QWidget *m_windowLayoutWidget;
    QStackedLayout *m_windowLayout;

    std::weak_ptr<CheckersPlayerNode> m_playerNode;

    bool m_connectedToServer;

    ChangeAccountPasswordFrame *m_changeAccountPasswordFrame;
    CreateAccountFrame *m_createAccountFrame;
    CreateLobbyFrame *m_createLobbyFrame;
    GameFrame *m_gameFrame;
    InLobbyFrame *m_inLobbyFrame;
    LobbyListFrame *m_lobbyListFrame;
    LobbyPasswordFrame *m_lobbyPasswordFrame;
    LogInAccountFrame *m_logInAccountFrame;
    MainMenuFrame *m_mainMenuFrame;
    StatisticsFrame *m_statisticsFrame;
    TitleFrame *m_titleFrame;

    bool m_showingDialog;
    DialogWidget *m_disconnectedDialog;
    DialogWidget *m_kickedDialog;
    DialogWidget *m_bannedDialog;
    DialogWidget *m_loggedOutDialog;
};

typedef std::unique_ptr<CheckersPlayerWindow> CheckersPlayerWindowUniPtr;
typedef std::weak_ptr<CheckersPlayerWindow> CheckersPlayerWindowWkPtr;