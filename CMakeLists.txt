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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

if (Qt5_POSITION_INDEPENDENT_CODE)
  SET(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

include_directories(include ${Qt5Widgets_INCLUDE_DIRS})

qt5_wrap_cpp(checkers_board_node_MOCS include/checkers_board_frame.hpp)

add_executable(checkers_board_node 
  src/checkers_board_frame.cpp 
  src/checkers_board_node.cpp
  ${checkers_board_node_MOCS})
ament_target_dependencies(checkers_board_node
  rclcpp
  std_msgs)
target_link_libraries(checkers_board_node
  Qt5::Widgets)

add_executable(checkers_player_node src/checkers_player_node.cpp)
ament_target_dependencies(checkers_player_node rclcpp std_msgs)

install(TARGETS
  checkers_board_node
  checkers_player_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following line skips the linter which checks for copyrights
  # Uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # The following line skips cpplint (only works in a git repo)
  # Uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()