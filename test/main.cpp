#include "executor.h"
#include "tasks.h"
#include "test.h"

#include "decorated_graph.h"

#include <algorithm>
#include <iostream>
#include <numeric>
#include <random>

using namespace anyf;

// std::this_thread::sleep_for (std::chrono::seconds(1));

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
  return make_pipeline<int, int>(create_vector, create_shuffle(seed),
                                 sort_vector, get_element(element));
}

// create  create    create  create
//   |        |        |       |
// double     |        |     double
//   |        |        |       |
// sum_vec sum_vec  sum_vec sum_vec
//    \      /         \       /
//     \    /           \     /
//      add               add
//       \                 /
//        -------   -------
//               add
//                |
//              result
auto create_graph() {
  return make_graph<int>({"size"})
      .add(create_pipeline(0, 10), "pipe1", {"size"})
      .add(create_pipeline(1, 10), "pipe2", {"size"})
      .add(create_pipeline(2, 10), "pipe3", {"size"})
      .add(create_pipeline(3, 10), "pipe4", {"size"})
      .add(create_pipeline(4, 10), "pipe5", {"size"})
      .add(create_pipeline(5, 10), "pipe6", {"size"})
      .add(create_pipeline(6, 10), "pipe7", {"size"})
      .add(create_pipeline(7, 10), "pipe8", {"size"})
      .add(sum, "sum1", {"pipe1", "pipe2"})
      .add(sum, "sum2", {"pipe3", "pipe4"})
      .add(sum, "sum3", {"pipe5", "pipe6"})
      .add(sum, "sum4", {"pipe7", "pipe8"})
      .add(sum, "sum5", {"sum1", "sum2"})
      .add(sum, "sum6", {"sum3", "sum4"})
      .add(sum, "final_sum", {"sum5", "sum6"})
      .output<int>(("final_sum"));
}

template <typename Out, typename F, typename... Inputs>
struct timing_decorator {
  Out operator()(F f, const std::string& name, Inputs... inputs) const {
    auto t0 = std::chrono::steady_clock::now();
    Out result = f(std::move(inputs)...);
    auto t1 = std::chrono::steady_clock::now();

    std::cout << name << " took "
              << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0)
                     .count()
              << "us\n";

    return result;
  }
};

auto create_decorated_graph() {
  return make_decorated_graph<timing_decorator, int>({"size"})
      .add(create_vector, "create", {"size"})
      .add(create_shuffle(0), "shuffle", {"create"})
      .add(sort_vector, "sort", {"shuffle"})
      .add(get_element(305), "get", {"sort"})
      .output<int>(("get"));
}

template <typename Graph>
void execute_graph(Graph g) {
  int size = 100'000;
  int max_threads = 4;

  for(int num_threads = 1; num_threads <= max_threads; num_threads++) {
    TaskSystem task_system(num_threads);

    auto t0 = std::chrono::steady_clock::now();
    std::cout << "--------- " << num_threads << " THREADS --------\n";
    uint64_t result = execute_graph(g, task_system, size);
    auto t1 = std::chrono::steady_clock::now();
    std::cout << "result is " << result << " after "
              << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0)
                     .count()
              << "us\n";
  }
}

} // namespace

int main() {
  execute_graph(create_graph());
  execute_graph(create_decorated_graph());
  // any_function_test();
  // stress_test();

  return 0;
}