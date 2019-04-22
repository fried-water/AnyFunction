#include "executor/sequential_executor.h"
#include "executor/task_executor.h"
#include "executor/tbb_executor.h"

#include "graph_execution.h"
#include "sentinal.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <numeric>
#include <random>

#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

using namespace anyf;

namespace {
std::vector<int> create_vector(int size) {
  std::vector<int> result;
  result.reserve(size);
  for(int i = 0; i < size; i++) {
    result.push_back(i);
  }

  return result;
}

auto create_shuffle(int seed) {
  return [seed](std::vector<int> vec) {
    std::shuffle(vec.begin(), vec.end(), std::mt19937(seed));

    return vec;
  };
}

auto get_element(int idx) {
  return [idx](std::vector<int> vec) { return vec[idx]; };
}

std::vector<int> sort_vector(std::vector<int> vec) {
  std::sort(vec.begin(), vec.end());

  return vec;
}

int sum(int x, int y) { return x + y; }

// any_function::func_type timing_decorator(std::string name,
//                                          any_function::func_type func) {

//   return [name = std::move(name),
//           func = std::move(func)](any_function::vec_ptr_type inputs) {
//     auto t0 = std::chrono::steady_clock::now();
//     std::any result = func(std::move(inputs));
//     auto t1 = std::chrono::steady_clock::now();

//     std::cout << name << " took "
//               << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0)
//                      .count()
//               << "us\n";

//     return result;
//   };
// }

// any_function::func_type print_decorator(std::string name,
//                                         any_function::func_type func) {
//   return [name = std::move(name),
//           func = std::move(func)](any_function::vec_ptr_type inputs) {
//     std::cout << "Running " << name << "\n";
//     return func(std::move(inputs));
//   };
// }

auto create_pipeline(int seed, int element) {
  return make_graph<int>({"size"})
    .add(create_vector, "create", {".size"})
    .add(create_shuffle(seed), "shuffle", {"create"})
    .add(sort_vector, "sort", {"shuffle"})
    .add(get_element(element), "get", {"sort"})
    .output<int>({"get"});
}

auto create_graph() {
  return make_graph<int>({"size"})
      .add(create_pipeline(0, 10), "pipe1", {".size"})
      .add(create_pipeline(1, 10), "pipe2", {".size"})
      .add(create_pipeline(2, 10), "pipe3", {".size"})
      .add(create_pipeline(3, 10), "pipe4", {".size"})
      .add(create_pipeline(4, 10), "pipe5", {".size"})
      .add(create_pipeline(5, 10), "pipe6", {".size"})
      .add(create_pipeline(6, 10), "pipe7", {".size"})
      .add(create_pipeline(7, 10), "pipe8", {".size"})
      .add(sum, "sum1", {"pipe1", "pipe2"})
      .add(sum, "sum2", {"pipe3", "pipe4"})
      .add(sum, "sum3", {"pipe5", "pipe6"})
      .add(sum, "sum4", {"pipe7", "pipe8"})
      .add(sum, "sum5", {"sum1", "sum2"})
      .add(sum, "sum6", {"sum3", "sum4"})
      .add(sum, "final_sum", {"sum5", "sum6"})
      .output<int>({"final_sum"});
}

template <typename Executor, typename Graph>
void execute_graph_with_threads(Graph g) {
  int size = 100'000;
  int max_threads = 4;

  for(int num_threads = 1; num_threads <= max_threads; num_threads++) {
    Executor executor(num_threads);

    auto decorated_graph = g;
    // .decorate(timing_decorator);
    // .decorate(print_decorator);

    auto t0 = std::chrono::steady_clock::now();
    std::cout << "--------- " << num_threads << " THREADS --------\n";
    uint64_t result = std::get<0>(execute_graph(decorated_graph, executor, size));
    auto t1 = std::chrono::steady_clock::now();
    std::cout << "result is " << result << " after "
              << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0)
                     .count()
              << "us\n";
  }
}

auto take(int i) {
  return [i](Sentinal sent) {
    std::cout << "Take" << i << " " << sent.copies << " " << sent.moves << "\n";
  };
}

void take_ref(const Sentinal& sent) {
  std::cout << "Ref "
            << " " << sent.copies << " " << sent.moves << "\n";
}

} // namespace

BOOST_AUTO_TEST_CASE(example_graph_tbb) {
  std::cout << "\nExecuting graph with TBB\n\n";
  execute_graph_with_threads<tbb_executor>(create_graph());
}

BOOST_AUTO_TEST_CASE(example_graph_seq) {
  std::cout << "\nExecuting graph Sequentially\n\n";
  execute_graph_with_threads<sequential_executor>(create_graph());
}

BOOST_AUTO_TEST_CASE(example_graph_task) {
  std::cout << "\nExecuting graph with custom task system\n\n";
  execute_graph_with_threads<task_executor>(create_graph());
}

// BOOST_AUTO_TEST_CASE(test_graph) {
//   auto g = make_graph<sentinal, sentinal>({"in1", "[nocopy]in2"})
//                .add(take(1), "val", {"in1"})
//                .add(take(2), "val2", {"in1"})
//                .add(take(3), "val3", {"in2"})
//                .add(take_ref, "ref", {"in2"})
//                .output<sentinal>("in2");

//   tbb_executor task_system;
//   sentinal x = execute_graph(g, task_system, sentinal(), sentinal());
//   std::cout << "Result "
//             << " " << x.copies << " " << x.moves << "\n";
// }
