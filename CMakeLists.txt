cmake_minimum_required(VERSION 3.15)

project(any_function)

find_package(Boost REQUIRED COMPONENTS unit_test_framework system)
find_package(tbb REQUIRED)

add_library(any_function INTERFACE)
target_compile_features(any_function INTERFACE cxx_std_17)
target_include_directories(any_function INTERFACE include ${BOOST_INCLUDE_DIRS})
target_link_libraries(any_function INTERFACE tbb)

#file(GLOB_RECURSE TEST_SOURCES test/*_test.cpp)

add_executable(tests
  test/test_main.cpp
  test/type_test.cpp
  test/any_function_test.cpp
  test/graph_test.cpp
  test/higher_order_test.cpp
  test/executor_test.cpp
  test/stress_test.cpp)

target_link_libraries(tests PRIVATE any_function ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
target_include_directories(tests PRIVATE test)
