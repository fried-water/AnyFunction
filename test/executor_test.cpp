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


auto create_pipeline(int seed, int element) {
  auto create = fg(create_vector);
  auto shuffle = fg(create_shuffle(seed));
  auto sort = fg(sort_vector);
  auto get_ele = fg(get_element(element));

  auto [g, size] =  make_graph<int>();
  return std::move(g).outputs<int>(get_ele(sort(shuffle(create(size)))));
}

auto create_graph() {
  auto [g, size] =  make_graph<int>();

  std::array<Edge<int>, 8> ps = {
    create_pipeline(0, 10)(size),
    create_pipeline(1, 10)(size),
    create_pipeline(2, 10)(size),
    create_pipeline(3, 10)(size),
    create_pipeline(4, 10)(size),
    create_pipeline(5, 10)(size),
    create_pipeline(6, 10)(size),
    create_pipeline(7, 10)(size)
  };

  auto fg_sum = fg(sum);
  return std::move(g).outputs<int>(fg_sum(fg_sum(fg_sum(ps[0], ps[1]), fg_sum(ps[2], ps[3])),
    fg_sum(fg_sum(ps[4], ps[5]), fg_sum(ps[6], ps[7]))));
}

template <typename Executor, typename Graph>
void execute_graph_with_threads(Graph g) {
  int size = 100'000;
  int max_threads = 8;

  for(int num_threads = 1; num_threads <= max_threads; num_threads++) {
    Executor executor(num_threads);

    auto t0 = std::chrono::steady_clock::now();
    uint64_t result = std::get<0>(execute_graph(g, executor, size));
    auto t1 = std::chrono::steady_clock::now();
    std::cout << num_threads << " THREADS: result is " << result << " after "
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


// BOOST_AUTO_TEST_CASE(example_graph_seq) {
//   auto g = fg([](int x) { return x;});

//   sequential_executor e;
//   int result = std::get<0>(execute_graph(g, e, 10));
//   std::cout << result << std::endl; 
// }

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
