cmake_minimum_required (VERSION 3.1)
project (ANY_FUNCTION)

#set(CMAKE_CXX_FLAGS "-Og -g -Wall -Wextra -fsanitize=address")
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra")

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost REQUIRED COMPONENTS unit_test_framework) 

add_executable (main
  test/test_main.cpp
  test/executor_test.cpp
  test/graph_test.cpp
  test/any_function_test.cpp
  test/stress_test.cpp)

target_include_directories(main PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
set_target_properties(main PROPERTIES CXX_STANDARD 17 CXX_EXTENSIONS OFF)
target_link_libraries (main pthread tbb ${Boost_LIBRARIES})

file(GLOB_RECURSE ALL_SOURCE_FILES test/*.cpp test/*.h include/*.h)

add_custom_target(format
  COMMAND /usr/bin/clang-format
  -style=file
  -i
  ${ALL_SOURCE_FILES}
)
