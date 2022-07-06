cmake_minimum_required(VERSION 3.15)

project(any_function_test)

find_package(Boost REQUIRED COMPONENTS unit_test_framework system)

add_executable(anyf_test
  test_main.cpp
  any_test.cpp
  any_function_test.cpp
  executor_test.cpp
  future_test.cpp
  graph_test.cpp
  static_graph_test.cpp
  stress_test.cpp
  type_test.cpp
  )

target_link_libraries(anyf_test PRIVATE any_function ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
target_include_directories(anyf_test PRIVATE test)

add_test(NAME AnyFunctionUnitTests COMMAND anyf_test)
