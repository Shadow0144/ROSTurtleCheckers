#include "shared/ErrorCode.hpp"

#include <unordered_map>
#include <string>

static std::unordered_map<ErrorCode::Code, std::string> s_errorCodeStringMap{
    {ErrorCode::Code::NONE, ""},
    {ErrorCode::Code::INCORRECT_PASSWORD, "Incorrect password"},
    {ErrorCode::Code::PLAYER_NOT_LOGGED_IN, "Player not logged in"},
    {ErrorCode::Code::PLAYER_ALREADY_LOGGED_IN, "Player already logged in"},
    {ErrorCode::Code::USERNAME_ALREADY_EXISTS, "Username already taken"},
    {ErrorCode::Code::USERNAME_DOES_NOT_EXIST, "Username not found"},
    {ErrorCode::Code::PLAYER_IS_BANNED, "This account has been banned"},
    {ErrorCode::Code::LOBBY_NAME_UNAVAILABLE, "Lobby name currently unavailable"},
    {ErrorCode::Code::LOBBY_ALREADY_EXISTS, "Lobby already exists"},
    {ErrorCode::Code::LOBBY_DOES_NOT_EXIST, "Lobby not found"},
    {ErrorCode::Code::LOBBY_IS_FULL, "Lobby is full"},
    {ErrorCode::Code::PLAYER_ALREADY_IN_LOBBY, "Player is already connected to lobby"},
    {ErrorCode::Code::DATABASE_ERROR, "Database error"}, // Do not send to player
    {ErrorCode::Code::SERVERSIDE_ERROR, "An error occurred on the server"}};

std::string ErrorCode::getErrorString(uint64_t errorCode)
{
    return getErrorString(static_cast<ErrorCode::Code>(errorCode));
}

std::string ErrorCode::getErrorString(ErrorCode::Code errorCode)
{
    return s_errorCodeStringMap[errorCode];
}