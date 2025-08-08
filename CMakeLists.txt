cmake_minimum_required(VERSION 3.5)
project(turtle_checkers)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(SQLite3 REQUIRED)
find_package(turtle_checkers_interfaces REQUIRED)

if (Qt5_POSITION_INDEPENDENT_CODE)
  SET(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

include_directories(include ${Qt5Widgets_INCLUDE_DIRS})

qt5_wrap_cpp(CheckersGameWindow_MOCS
  include/player/TitleWidget.hpp
  include/player/DialogWidget.hpp
  include/player/LobbyDetailsWidget.hpp
  include/player/MatchDetailsWidget.hpp
  include/player/TileRender.hpp
  include/player/TurtlePieceRender.hpp
  include/player/CheckersBoardRender.hpp
  include/player/frame/CreateAccountFrame.hpp
  include/player/frame/CreateLobbyFrame.hpp
  include/player/frame/GameFrame.hpp
  include/player/frame/InLobbyFrame.hpp
  include/player/frame/LobbyListFrame.hpp
  include/player/frame/LobbyPasswordFrame.hpp
  include/player/frame/LogInAccountFrame.hpp
  include/player/frame/MainMenuFrame.hpp
  include/player/frame/StatisticsFrame.hpp
  include/player/frame/TitleFrame.hpp
  include/player/CheckersPlayerWindow.hpp 
  include/player/CheckersPlayerNode.hpp)

add_executable(checkers_player_node
  src/shared/RSAKeyGenerator.cpp
  src/player/ImageLibrary.cpp
  src/shared/Tile.cpp
  src/shared/TurtleLogger.cpp
  src/shared/TurtlePiece.cpp
  src/player/Parameters.cpp
  src/player/TileRender.cpp
  src/player/TileRenderFactory.cpp
  src/player/TurtlePieceRender.cpp
  src/player/TurtlePieceRenderFactory.cpp
  src/player/TurtleGraveyard.cpp
  src/player/ChatBox.cpp
  src/player/HUD.cpp
  src/player/CheckersBoardRender.cpp
  src/player/TitleWidget.cpp
  src/player/DialogWidget.cpp
  src/player/LobbyDetailsWidget.cpp
  src/player/MatchDetailsWidget.cpp
  src/player/frame/CreateAccountFrame.cpp
  src/player/frame/CreateLobbyFrame.cpp
  src/player/frame/GameFrame.cpp
  src/player/frame/InLobbyFrame.cpp
  src/player/frame/LobbyListFrame.cpp
  src/player/frame/LobbyPasswordFrame.cpp
  src/player/frame/LogInAccountFrame.cpp
  src/player/frame/MainMenuFrame.cpp
  src/player/frame/StatisticsFrame.cpp
  src/player/frame/TitleFrame.cpp
  src/player/CheckersPlayerWindow.cpp
  src/player/CheckersPlayerNode.cpp
  ${CheckersGameWindow_MOCS})
ament_target_dependencies(checkers_player_node
  ament_index_cpp
  rclcpp
  turtle_checkers_interfaces)
target_link_libraries(checkers_player_node
  Qt5::Widgets)

add_executable(checkers_game_master_node
  src/shared/RSAKeyGenerator.cpp
  src/shared/Tile.cpp
  src/shared/TurtleLogger.cpp
  src/shared/TurtlePiece.cpp
  src/game_master/DatabaseHandler.cpp
  src/game_master/TileFactory.cpp
  src/game_master/TurtlePieceFactory.cpp
  src/game_master/MasterBoard.cpp
  src/game_master/CheckersGameLobby.cpp
  src/game_master/CheckersGameMasterNode.cpp)
ament_target_dependencies(checkers_game_master_node
  ament_index_cpp
  rclcpp
  turtle_checkers_interfaces)
target_link_libraries(checkers_game_master_node
  SQLite::SQLite3)

install(TARGETS
  checkers_player_node
  checkers_game_master_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(FILES turtles_checkers_authorization_key.key
  DESTINATION share/${PROJECT_NAME})

install(FILES report_email.config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY img
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.png")

install(DIRECTORY styles
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.qss")
  
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()