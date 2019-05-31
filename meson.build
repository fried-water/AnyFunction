project('any_function', 'cpp', 
  default_options : ['cpp_std=c++17', 'buildtype=release',
    'warnlevel=3'])

header_files = [
  'include/any_function.h',
  'include/graph.h',
  'include/graph_execution.h',
  'include/higher_order.h',
  'include/traits.h',
  'include/type.h',
  'include/util.h',
  'include/executor/sequential_executor.h',
  'include/executor/task_executor.h',
  'include/executor/tbb_executor.h']

cpp_files = [
  'test/test_main.cpp',
  'test/type_test.cpp',
  'test/any_function_test.cpp',
  'test/graph_test.cpp',
  'test/higher_order_test.cpp',
  'test/executor_test.cpp',
  'test/stress_test.cpp'
]

local_header_files = [
  'test/sentinal.h']

incdir = include_directories('include')

tbb = dependency('tbb')
thread = dependency('threads')
boost = dependency('boost', modules : ['unit_test_framework'])

executable('anyf_tests', 
  cpp_files,
  dependencies : [tbb, boost, thread], 
  include_directories : incdir,
  cpp_args : ['-Wextra'])

run_target('format',
  command : ['clang-format', '-i'] + header_files + cpp_files + local_header_files)
