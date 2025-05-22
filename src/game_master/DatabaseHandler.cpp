
#include "game_master/DatabaseHandler.hpp"

#include <sqlite3.h>

#include <string>
#include <filesystem>
#include <random>

#include "shared/CheckersConsts.hpp"
#include "shared/RSAKeyGenerator.hpp"
#include "shared/TurtleLogger.hpp"

static const std::string s_chars =
    "0123456789"
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz";

static std::mt19937 s_rng(std::random_device{}()); // Random engine
static std::uniform_int_distribution<> s_dist(0, s_chars.size() - 1);

constexpr size_t ID_INDEX = 0u;
constexpr size_t NAME_INDEX = 1u;
constexpr size_t PASSWORD_HASH_INDEX = 2u;
constexpr size_t PASSWORD_SALT_INDEX = 3u;
constexpr size_t MATCHES_PLAYED_INDEX = 4u;
constexpr size_t MATCHES_WON_INDEX = 5u;
constexpr size_t MATCHES_LOST_INDEX = 6u;
constexpr size_t MATCHES_DRAWED_INDEX = 7u;
constexpr size_t TIME_PLAYED_INDEX = 8u;
constexpr size_t BANNED_INDEX = 9u;

DatabaseHandler::DatabaseHandler(const std::string &databaseDir)
{
    m_db = nullptr;
    std::filesystem::path dir = databaseDir;
    std::filesystem::path dbFile = "turtle_checkers.db";
    if (!std::filesystem::exists(dir))
    {
        std::filesystem::create_directory(dir);
    }
    if (sqlite3_open((dir / dbFile).c_str(), &m_db) == SQLITE_OK)
    {
        auto query = "CREATE TABLE IF NOT EXISTS users ("
                     "id INTEGER PRIMARY KEY AUTOINCREMENT,"
                     "name TEXT UNIQUE NOT NULL,"
                     "password_hash TEXT NOT NULL,"
                     "password_salt TEXT NOT NULL,"
                     "matches_played INTEGER,"
                     "matches_won INTEGER,"
                     "matches_lost INTEGER,"
                     "matches_drawed INTEGER,"
                     "banned INTEGER);";

        char *errMsg = nullptr;
        auto result = sqlite3_exec(m_db, query, nullptr, nullptr, &errMsg);
        if (result != SQLITE_OK)
        {
            TurtleLogger::logWarn("SQL error: " + std::string(errMsg));
            sqlite3_free(errMsg);
        }
        else
        {
            TurtleLogger::logInfo("Player database created/loaded successfully");
        }
    }
    else
    {
        TurtleLogger::logWarn("Could not create/load player database: " + std::string(sqlite3_errmsg(m_db)));
        sqlite3_close(m_db);
    }
}

DatabaseHandler::~DatabaseHandler()
{
    sqlite3_close(m_db);
}

bool DatabaseHandler::checkPlayerExists(const std::string &playerName)
{
    bool exists = false;

    // Prepare the query
    auto query = "SELECT EXISTS (SELECT 1 FROM users WHERE (name = ?));";
    sqlite3_stmt *stmt = nullptr;
    if (sqlite3_prepare_v2(m_db, query, -1, &stmt, nullptr) != SQLITE_OK)
    {
        m_errorMessage = "Failed to prepare SQL query";
        TurtleLogger::logWarn("Failed to prepare SQL query");
        return false;
    }

    // Fill in the values
    sqlite3_bind_text(stmt, 1, playerName.c_str(), -1, SQLITE_TRANSIENT);

    // Run the query
    if (sqlite3_step(stmt) != SQLITE_ROW)
    {
        m_errorMessage = "Failed to run SQL query";
        TurtleLogger::logWarn("Failed to run SQL query: " + std::string(sqlite3_errmsg(m_db)));
        return false;
    }

    // Check if a row was found
    exists = (sqlite3_column_int(stmt, 0) == 1); // 1 for true

    // Free the resources
    sqlite3_finalize(stmt);

    return exists;
}

bool DatabaseHandler::addPlayer(const std::string &playerName, size_t hashedPlayerPassword)
{
    if (!m_db)
    {
        m_errorMessage = "Failed to access player database";
        TurtleLogger::logWarn("SQL database not found");
        return false;
    }

    // Try to find if the player name already exists in the database
    if (checkPlayerExists(playerName))
    {
        m_errorMessage = "User name already in use";
        TurtleLogger::logWarn("User name already in use");
        return false;
    }

    // Prepare the query
    auto query = "INSERT INTO users (name, password_hash, password_salt, "
                 "matches_played, matches_won, matches_lost, matches_drawed, banned) "
                 "VALUES (?, ?, ?, 0, 0, 0, 0, 0);";
    sqlite3_stmt *stmt = nullptr;
    if (sqlite3_prepare_v2(m_db, query, -1, &stmt, nullptr) != SQLITE_OK)
    {
        m_errorMessage = "Failed to prepare SQL query";
        TurtleLogger::logWarn("Failed to prepare SQL query");
        return false;
    }

    // Generate a salt
    auto passwordSalt = generateRandomString(NUM_CHARS_SALT);
    auto saltedPlayerPasswordRehash = std::to_string(RSAKeyGenerator::hashString(std::to_string(hashedPlayerPassword) + passwordSalt));

    // Fill in the values
    sqlite3_bind_text(stmt, 1, playerName.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, saltedPlayerPasswordRehash.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 3, passwordSalt.c_str(), -1, SQLITE_TRANSIENT);

    // Run the query
    if (sqlite3_step(stmt) != SQLITE_DONE)
    {
        m_errorMessage = "Failed to run SQL query";
        TurtleLogger::logWarn("Failed to run SQL query: " + std::string(sqlite3_errmsg(m_db)));
        sqlite3_finalize(stmt);
        return false;
    }

    // Free the resources
    sqlite3_finalize(stmt);

    // Success!
    TurtleLogger::logInfo("Created account: " + playerName);

    return true;
}

bool DatabaseHandler::removePlayer(const std::string &playerName)
{
    if (!m_db)
    {
        m_errorMessage = "Failed to access player database";
        TurtleLogger::logWarn("SQL database not found");
        return false;
    }

    // Try to find if the player name exists in the database
    if (!checkPlayerExists(playerName))
    {
        m_errorMessage = "User name not found";
        TurtleLogger::logWarn("User name not found");
        return false;
    }

    // Prepare the query
    auto query = "DELETE FROM users WHERE (name = ?);";
    sqlite3_stmt *stmt = nullptr;
    if (sqlite3_prepare_v2(m_db, query, -1, &stmt, nullptr) != SQLITE_OK)
    {
        m_errorMessage = "Failed to prepare SQL query";
        TurtleLogger::logWarn("Failed to prepare SQL query");
        return false;
    }

    // Fill in the values
    sqlite3_bind_text(stmt, 1, playerName.c_str(), -1, SQLITE_TRANSIENT);

    // Run the query
    if (sqlite3_step(stmt) != SQLITE_ROW)
    {
        m_errorMessage = "Failed to run SQL query";
        TurtleLogger::logWarn("Failed to run SQL query: " + std::string(sqlite3_errmsg(m_db)));
        sqlite3_finalize(stmt);
        return false;
    }

    return true;
}

bool DatabaseHandler::checkPasswordCorrect(const std::string &playerName, size_t hashedPlayerPassword)
{
    if (!m_db)
    {
        m_errorMessage = "Failed to access player database";
        TurtleLogger::logWarn("SQL database not found");
        return false;
    }

    // Try to find if the player name exists in the database
    if (!checkPlayerExists(playerName))
    {
        m_errorMessage = "User name not found";
        TurtleLogger::logWarn("User name not found");
        return false;
    }

    // Prepare the query
    auto query = "SELECT * FROM users WHERE (name = ?);";
    sqlite3_stmt *stmt = nullptr;
    if (sqlite3_prepare_v2(m_db, query, -1, &stmt, nullptr) != SQLITE_OK)
    {
        m_errorMessage = "Failed to prepare SQL query";
        TurtleLogger::logWarn("Failed to prepare SQL query");
        return false;
    }

    // Fill in the values
    sqlite3_bind_text(stmt, 1, playerName.c_str(), -1, SQLITE_TRANSIENT);

    // Run the query
    if (sqlite3_step(stmt) != SQLITE_ROW)
    {
        m_errorMessage = "Failed to run SQL query";
        TurtleLogger::logWarn("Failed to run SQL query: " + std::string(sqlite3_errmsg(m_db)));
        sqlite3_finalize(stmt);
        return false;
    }

    // Get the values from the columns
    auto storedSaltedPasswordHashPtr = sqlite3_column_text(stmt, PASSWORD_HASH_INDEX);
    std::string storedSaltedPasswordHash = storedSaltedPasswordHashPtr ? reinterpret_cast<const char *>(storedSaltedPasswordHashPtr) : "";
    auto passwordSaltPtr = sqlite3_column_text(stmt, PASSWORD_SALT_INDEX);
    std::string passwordSalt = passwordSaltPtr ? reinterpret_cast<const char *>(passwordSaltPtr) : "";

    // Add the salt, hash again, and check if it matches what's stored
    auto saltedPlayerPasswordRehash = std::to_string(RSAKeyGenerator::hashString(std::to_string(hashedPlayerPassword) + passwordSalt));
    bool match = (saltedPlayerPasswordRehash == storedSaltedPasswordHash);

    if (!match)
    {
        m_errorMessage = "Incorrect password";
    }

    // Free the resources
    sqlite3_finalize(stmt);

    return match;
}

const std::string DatabaseHandler::getErrorMessage()
{
    auto message = m_errorMessage;
    m_errorMessage.clear();
    return message;
}

const std::string DatabaseHandler::generateRandomString(size_t length)
{
    std::string result;
    result.reserve(length);
    for (size_t i = 0u; i < length; ++i)
    {
        result += s_chars[s_dist(s_rng)];
    }
    return result;
}