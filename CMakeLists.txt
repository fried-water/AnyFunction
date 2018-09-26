cmake_minimum_required (VERSION 2.8.11)
project (ANY_FUNCTION)

set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra")

add_executable (main test/main.cpp)

target_include_directories (main PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

set_target_properties(main PROPERTIES
            CXX_STANDARD 17
            CXX_EXTENSIONS OFF
            )