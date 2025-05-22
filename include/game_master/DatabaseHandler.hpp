#pragma once

// #ifndef Q_MOC_RUN			  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
//  NO LINT
// #endif

#include <sqlite3.h>

#include <string>
#include <memory>

class DatabaseHandler
{
public:
    DatabaseHandler(const std::string &databaseDir);
    ~DatabaseHandler();

    bool checkPlayerExists(const std::string &playerName);

    bool addPlayer(const std::string &playerName, size_t hashedPlayerPassword);
    bool removePlayer(const std::string &playerName);

    bool checkPasswordCorrect(const std::string &playerName, size_t hashedPlayerPassword);

    const std::string getErrorMessage(); // Clears the message after getting

private:
    const std::string generateRandomString(size_t length);

    sqlite3 *m_db;

    std::string m_errorMessage;
};

typedef std::unique_ptr<DatabaseHandler> DatabaseHandlerUniPtr;