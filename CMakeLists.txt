cmake_minimum_required(VERSION 3.15)

project(any_function)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "Setting build type to Release as none was specified.")
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(tbb REQUIRED)
find_package(fmt REQUIRED)

add_subdirectory(ext/knot)

set(CMAKE_CXX_STANDARD 17)
set(EXPECTED_BUILD_TESTS OFF)
add_subdirectory(ext/expected)

add_library(any_function INTERFACE)

target_compile_features(any_function INTERFACE cxx_std_17)
target_include_directories(any_function INTERFACE include)
target_link_libraries(any_function INTERFACE tbb fmt knot expected)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  enable_testing()
  add_subdirectory(test)
endif()
