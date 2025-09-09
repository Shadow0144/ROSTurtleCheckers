#pragma once

#include <unordered_map>
#include <string>

class ErrorCode
{
public:
    enum class Code
    {
        NONE,
        INCORRECT_PASSWORD,
        PLAYER_NOT_LOGGED_IN,
        PLAYER_ALREADY_LOGGED_IN,
        USERNAME_ALREADY_EXISTS,
        USERNAME_DOES_NOT_EXIST,
        PLAYER_IS_BANNED,
        LOBBY_NAME_UNAVAILABLE,
        LOBBY_ALREADY_EXISTS,
        LOBBY_DOES_NOT_EXIST,
        LOBBY_IS_FULL,
        PLAYER_ALREADY_IN_LOBBY,
        DATABASE_ERROR,
        SERVERSIDE_ERROR
    };

    static std::string getErrorString(uint64_t errorCode);
    static std::string getErrorString(ErrorCode::Code errorCode);
};