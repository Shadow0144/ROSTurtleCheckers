cmake_minimum_required(VERSION 3.5)
project(turtle_checkers)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(turtle_checkers_interfaces REQUIRED)

if (Qt5_POSITION_INDEPENDENT_CODE)
  SET(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

include_directories(include ${Qt5Widgets_INCLUDE_DIRS})

qt5_wrap_cpp(CheckersGameWindow_MOCS 
  include/player/TileRender.hpp
  include/player/TurtlePieceRender.hpp
  include/player/CheckersBoardRender.hpp
  include/player/CheckersMainMenuFrame.hpp
  include/player/CheckersGameFrame.hpp
  include/player/CheckersPlayerWindow.hpp 
  include/player/CheckersPlayerNode.hpp)

add_executable(checkers_player_node 
  src/shared/RSAKeyGenerator.cpp
  src/player/ImageLibrary.cpp
  src/shared/Tile.cpp
  src/player/TileRender.cpp
  src/player/TileRenderFactory.cpp
  src/shared/TurtlePiece.cpp
  src/player/TurtlePieceRender.cpp
  src/player/TurtlePieceRenderFactory.cpp
  src/player/TurtleGraveyard.cpp
  src/player/HUD.cpp
  src/player/CheckersBoardRender.cpp
  src/player/CheckersMainMenuFrame.cpp 
  src/player/CheckersGameFrame.cpp 
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
  src/game_master/TileFactory.cpp
  src/shared/TurtlePiece.cpp
  src/game_master/TurtlePieceFactory.cpp
  src/game_master/MasterBoard.cpp
  src/game_master/CheckersGameLobby.cpp
  src/game_master/CheckersGameMasterNode.cpp)
ament_target_dependencies(checkers_game_master_node
  rclcpp 
  turtle_checkers_interfaces)

install(TARGETS
  checkers_player_node
  checkers_game_master_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY img
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.png")

  install(DIRECTORY styles
    DESTINATION share/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.qss")
  
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()