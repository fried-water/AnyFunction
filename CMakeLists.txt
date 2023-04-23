cmake_minimum_required(VERSION 3.15)

project(any_function)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "Setting build type to Release as none was specified.")
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(tbb REQUIRED)
find_package(fmt REQUIRED)

add_subdirectory(ext/knot)

set(EXPECTED_BUILD_TESTS OFF)
add_subdirectory(ext/expected)

add_library(any_function STATIC
  src/graph.cpp
  src/graph_execution.cpp)

target_compile_features(any_function PUBLIC cxx_std_17)
target_include_directories(any_function PUBLIC include PRIVATE src)
target_link_libraries(any_function PUBLIC tbb fmt knot expected)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_compile_options(any_function PUBLIC -Og -g -fsanitize=address)
  target_link_options(any_function PUBLIC -Og -g -fsanitize=address)
endif()

  enable_testing()
  add_subdirectory(test)
endif()
