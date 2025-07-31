#pragma once

// #ifndef Q_MOC_RUN			  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
//  NO LINT
// #endif

#include <sqlite3.h>

#include <string>
#include <memory>
#include <vector>

struct PlayerStatistics
{
    std::string playerName;
    struct MatchInfo
    {
        std::string lobbyNameId;
        std::string blackPlayerName;
        std::string redPlayerName;
        int winner;
    };
    std::vector<MatchInfo> matchInfoList;
    int matchesPlayed;
    int matchesWon;
    int matchesLost;
    int matchesDrawed;
};

class DatabaseHandler
{
public:
    DatabaseHandler(const std::string &databaseDir);
    ~DatabaseHandler();

    bool checkPlayerExists(const std::string &playerName);

    bool addPlayer(const std::string &playerName, size_t hashedPlayerPassword);
    bool removePlayer(const std::string &playerName);

    bool checkPlayerBanned(const std::string &playerName);
    bool setPlayerBanned(const std::string &playerName, bool banned);

    bool checkPasswordCorrect(const std::string &playerName, size_t hashedPlayerPassword);

    bool changePassword(const std::string &playerName, size_t hashedPlayerPassword);

    bool addMatch(const std::string &lobbyName, const std::string &lobbyId,
                  const std::string &blackPlayerName, const std::string &redPlayerName,
                  int winner);

    PlayerStatistics getPlayerStatistics(const std::string &playerName);

    const std::string getErrorMessage(); // Clears the message after getting

private:
    const std::string generateRandomString(size_t length);

    sqlite3 *m_db;

    std::string m_errorMessage;
};

typedef std::unique_ptr<DatabaseHandler> DatabaseHandlerUniPtr;