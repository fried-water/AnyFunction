cmake_minimum_required(VERSION 3.15)

project(any_function)

find_package(tbb REQUIRED)
find_package(fmt REQUIRED)

add_library(any_function INTERFACE)

target_compile_features(any_function INTERFACE cxx_std_17)
target_include_directories(any_function INTERFACE include)
target_link_libraries(any_function INTERFACE tbb fmt)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  enable_testing()
  add_subdirectory(test)
endif()
