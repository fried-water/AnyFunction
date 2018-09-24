# CMakeLists files in this project can
# refer to the root source directory of the project as ${HELLO_SOURCE_DIR} and
# to the root binary directory of the project as ${HELLO_BINARY_DIR}.
cmake_minimum_required (VERSION 2.8.11)
project (ANY_FUNCTION)

set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra")

# Add executable called "helloDemo" that is built from the source files
# "demo.cxx" and "demo_b.cxx". The extensions are automatically found.
add_executable (main test/main.cpp)

target_include_directories (main PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
#target_include_directories (main PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/ext)

set_target_properties(main PROPERTIES
            CXX_STANDARD 17
            CXX_EXTENSIONS OFF
            )
