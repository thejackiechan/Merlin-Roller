cmake_minimum_required(VERSION 3.0.0)
project(MerlinRoller)

set(CMAKE_CXX_STANDARD 17)  # enable C++17 standard

include_directories(include)
set(SOURCES src/ball.cpp src/slope.cpp src/linalg.cpp src/physics.cpp src/simulation.cpp src/main.cpp)
add_executable(${PROJECT_NAME} ${SOURCES}) 


