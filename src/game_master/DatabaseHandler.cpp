
#include "game_master/DatabaseHandler.hpp"

#include <sqlite3.h>

#include <string>
#include <iostream>
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

// Player table
constexpr size_t PLAYER_ID_INDEX = 0u;
constexpr size_t NAME_INDEX = 1u;
constexpr size_t PASSWORD_HASH_INDEX = 2u;
constexpr size_t PASSWORD_SALT_INDEX = 3u;
constexpr size_t BANNED_INDEX = 4u;

// Matches table
constexpr size_t MATCHES_ID_INDEX = 0u;
constexpr size_t LOBBY_NAME_ID_INDEX = 1u;
constexpr size_t BLACK_PLAYER_NAME_INDEX = 2u;
constexpr size_t RED_PLAYER_NAME_INDEX = 3u;
constexpr size_t WINNER_INDEX = 4u;

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
        auto players_table_create_query = "CREATE TABLE IF NOT EXISTS players ("
                                          "id INTEGER PRIMARY KEY AUTOINCREMENT,"
                                          "name TEXT UNIQUE NOT NULL,"
                                          "password_hash TEXT NOT NULL,"
                                          "password_salt TEXT NOT NULL,"
                                          "banned INTEGER);";

        char *errMsg = nullptr;
        auto result = sqlite3_exec(m_db, players_table_create_query, nullptr, nullptr, &errMsg);
        if (result != SQLITE_OK)
        {
            TurtleLogger::logWarn("SQL error: " + std::string(errMsg));
            sqlite3_free(errMsg);
        }
        else
        {
            TurtleLogger::logInfo("Player database created/loaded successfully");
        }

        auto matches_table_create_query = "CREATE TABLE IF NOT EXISTS matches ("
                                          "id INTEGER PRIMARY KEY AUTOINCREMENT,"
                                          "lobby_name_id TEXT NOT NULL,"
                                          "black_player_name TEXT NOT NULL,"
                                          "red_player_name TEXT NOT NULL,"
                                          "winner INTEGER);";

        errMsg = nullptr;
        result = sqlite3_exec(m_db, matches_table_create_query, nullptr, nullptr, &errMsg);
        if (result != SQLITE_OK)
        {
            TurtleLogger::logWarn("SQL error: " + std::string(errMsg));
            sqlite3_free(errMsg);
        }
        else
        {
            TurtleLogger::logInfo("Matches database created/loaded successfully");
        }
    }
    else
    {
        TurtleLogger::logWarn("Could not create/load database: " + std::string(sqlite3_errmsg(m_db)));
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
    auto query = "SELECT EXISTS (SELECT 1 FROM players WHERE (name = ?));";
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
    auto query = "INSERT INTO players (name, password_hash, password_salt, banned) "
                 "VALUES (?, ?, ?, 0);";
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
    auto query = "DELETE FROM players WHERE (name = ?);";
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

bool DatabaseHandler::checkPlayerBanned(const std::string &playerName)
{
    bool banned = false;

    // Prepare the query
    auto query = "SELECT banned FROM players WHERE (name = ?);";
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
    banned = (sqlite3_column_int(stmt, 0) != 0); // 0 for false

    // Free the resources
    sqlite3_finalize(stmt);

    return banned;
}

bool DatabaseHandler::setPlayerBanned(const std::string &playerName, bool banned)
{
    // Prepare the query
    auto query = "UPDATE players SET banned = ? WHERE name = ?;";
    sqlite3_stmt *stmt = nullptr;
    if (sqlite3_prepare_v2(m_db, query, -1, &stmt, nullptr) != SQLITE_OK)
    {
        m_errorMessage = "Failed to prepare SQL query";
        TurtleLogger::logWarn("Failed to prepare SQL query");
        return false;
    }

    // Fill in the values
    sqlite3_bind_int(stmt, 1, static_cast<int>(banned));
    sqlite3_bind_text(stmt, 2, playerName.c_str(), -1, SQLITE_TRANSIENT);

    // Run the query
    if (sqlite3_step(stmt) != SQLITE_DONE)
    {
        m_errorMessage = "Failed to run SQL query";
        TurtleLogger::logWarn("Failed to run SQL query: " + std::string(sqlite3_errmsg(m_db)));
        return false;
    }

    // Free the resources
    sqlite3_finalize(stmt);

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
    auto query = "SELECT * FROM players WHERE (name = ?);";
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

bool DatabaseHandler::changePassword(const std::string &playerName, size_t hashedPlayerPassword)
{
    // Prepare the query
    auto query = "UPDATE players SET password_hash = ?, password_salt = ? WHERE name = ?;";
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
    sqlite3_bind_text(stmt, 1, saltedPlayerPasswordRehash.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, passwordSalt.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 3, playerName.c_str(), -1, SQLITE_TRANSIENT);

    // Run the query
    if (sqlite3_step(stmt) != SQLITE_DONE)
    {
        m_errorMessage = "Failed to run SQL query";
        TurtleLogger::logWarn("Failed to run SQL query: " + std::string(sqlite3_errmsg(m_db)));
        return false;
    }

    // Free the resources
    sqlite3_finalize(stmt);

    return true;
}

bool DatabaseHandler::addMatch(const std::string &lobbyName, const std::string &lobbyId,
                               const std::string &blackPlayerName, const std::string &redPlayerName,
                               int winner)
{
    if (!m_db)
    {
        m_errorMessage = "Failed to access player database";
        TurtleLogger::logWarn("SQL database not found");
        return false;
    }

    // Prepare the query
    auto query = "INSERT INTO matches (lobby_name_id, black_player_name, red_player_name, winner) "
                 "VALUES (?, ?, ?, ?);";
    sqlite3_stmt *stmt = nullptr;
    if (sqlite3_prepare_v2(m_db, query, -1, &stmt, nullptr) != SQLITE_OK)
    {
        m_errorMessage = "Failed to prepare SQL query";
        TurtleLogger::logWarn("Failed to prepare SQL query");
        return false;
    }

    const auto lobbyNameId = lobbyName + "#" + lobbyId;

    // Fill in the values
    sqlite3_bind_text(stmt, 1, lobbyNameId.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, blackPlayerName.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 3, redPlayerName.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 4, winner);

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
    TurtleLogger::logInfo("Created match record: " + lobbyNameId);

    return true;
}

PlayerStatistics DatabaseHandler::getPlayerStatistics(const std::string &playerName)
{
    PlayerStatistics playerStatistics;

    playerStatistics.playerName = playerName;

    // Prepare the query
    auto query = "SELECT * FROM matches WHERE (black_player_name = :PLAYER_NAME OR red_player_name = :PLAYER_NAME);";
    sqlite3_stmt *stmt = nullptr;
    if (sqlite3_prepare_v2(m_db, query, -1, &stmt, nullptr) != SQLITE_OK)
    {
        m_errorMessage = "Failed to prepare SQL query";
        TurtleLogger::logWarn("Failed to prepare SQL query");
        return PlayerStatistics{};
    }

    // Fill in the values
    int idx = sqlite3_bind_parameter_index(stmt, ":PLAYER_NAME");
    sqlite3_bind_text(stmt, idx, playerName.c_str(), -1, SQLITE_TRANSIENT);

    // Add the matches to the player statistics
    while (sqlite3_step(stmt) == SQLITE_ROW)
    {
        auto lobbyNameIdPtr = sqlite3_column_text(stmt, LOBBY_NAME_ID_INDEX);
        std::string lobbyNameId = lobbyNameIdPtr ? reinterpret_cast<const char *>(lobbyNameIdPtr) : "";

        auto blackPlayerNamePtr = sqlite3_column_text(stmt, BLACK_PLAYER_NAME_INDEX);
        std::string blackPlayerName = blackPlayerNamePtr ? reinterpret_cast<const char *>(blackPlayerNamePtr) : "";

        auto redPlayerNamePtr = sqlite3_column_text(stmt, RED_PLAYER_NAME_INDEX);
        std::string redPlayerName = redPlayerNamePtr ? reinterpret_cast<const char *>(redPlayerNamePtr) : "";

        auto winner = sqlite3_column_int(stmt, WINNER_INDEX);

        playerStatistics.matchInfoList.push_back(
            PlayerStatistics::MatchInfo{lobbyNameId, blackPlayerName, redPlayerName, winner});
    }

    // Free the resources
    sqlite3_finalize(stmt);

    // Prepare the query
    query = "SELECT "
            " SUM(CASE "
            "  WHEN black_player_name = :PLAYER_NAME OR red_player_name = :PLAYER_NAME "
            "  THEN 1 ELSE 0 END) AS matches_played, "
            " SUM(CASE "
            "  WHEN(black_player_name = :PLAYER_NAME AND winner = 1) "
            "  OR(red_player_name = :PLAYER_NAME AND winner = 2) "
            "  THEN 1 ELSE 0 END) AS matches_won, "
            " SUM(CASE "
            "  WHEN(black_player_name = :PLAYER_NAME AND winner = 2) "
            "  OR(red_player_name = :PLAYER_NAME AND winner = 1) "
            "  THEN 1 ELSE 0 END) AS matches_lost, "
            " SUM(CASE "
            "  WHEN(black_player_name = :PLAYER_NAME OR red_player_name = :PLAYER_NAME) "
            "  AND winner = 3 THEN 1 ELSE 0 END) AS matches_drawn "
            "FROM matches;";
    stmt = nullptr;
    if (sqlite3_prepare_v2(m_db, query, -1, &stmt, nullptr) != SQLITE_OK)
    {
        m_errorMessage = "Failed to prepare SQL query";
        TurtleLogger::logWarn("Failed to prepare SQL query");
        return PlayerStatistics{};
    }

    // Fill in the values
    idx = sqlite3_bind_parameter_index(stmt, ":PLAYER_NAME");
    sqlite3_bind_text(stmt, idx, playerName.c_str(), -1, SQLITE_TRANSIENT);

    // Run the query
    if (sqlite3_step(stmt) != SQLITE_ROW)
    {
        m_errorMessage = "Failed to run SQL query";
        TurtleLogger::logWarn("Failed to run SQL query: " + std::string(sqlite3_errmsg(m_db)));
        return PlayerStatistics{};
    }

    // Add the summaries to the player statistics
    playerStatistics.matchesPlayed = (sqlite3_column_type(stmt, 0) == SQLITE_NULL) ? 0 : sqlite3_column_int(stmt, 0);
    playerStatistics.matchesWon = (sqlite3_column_type(stmt, 1) == SQLITE_NULL) ? 0 : sqlite3_column_int(stmt, 1);
    playerStatistics.matchesLost = (sqlite3_column_type(stmt, 2) == SQLITE_NULL) ? 0 : sqlite3_column_int(stmt, 2);
    playerStatistics.matchesDrawn = (sqlite3_column_type(stmt, 3) == SQLITE_NULL) ? 0 : sqlite3_column_int(stmt, 3);

    // Free the resources
    sqlite3_finalize(stmt);

    return playerStatistics;
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