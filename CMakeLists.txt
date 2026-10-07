cmake_minimum_required(VERSION 3.16)
project(Physics2DEngine VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)
include_directories(sfml/include)

link_directories(sfml/lib)

add_executable(Physics2DEngine src/main.cpp)

target_link_libraries(Physics2DEngine
    sfml-graphics
    sfml-window
    sfml-system
)