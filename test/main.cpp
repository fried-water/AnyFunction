#include "executor.h"
#include "tasks.h"
#include "test.h"

#include <algorithm>
#include <iostream>
#include <numeric>
#include <random>

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

auto timing_decorator(std::string name, any_value_function func) {
  return std::function<std::any(any_value_function::vec_ptr_type &&)>(
      [name = std::move(name),
       func = std::move(func)](any_value_function::vec_ptr_type&& inputs) {
        auto t0 = std::chrono::steady_clock::now();
        std::any result = func.invoke(std::move(inputs));
        auto t1 = std::chrono::steady_clock::now();

        std::cout << name << " took "
                  << std::chrono::duration_cast<std::chrono::microseconds>(t1 -
                                                                           t0)
                         .count()
                  << "us\n";

        return result;
      });
}

auto print_decorator(std::string name, any_value_function func) {
  return std::function<std::any(any_value_function::vec_ptr_type &&)>(
      [name = std::move(name),
       func = std::move(func)](any_value_function::vec_ptr_type&& inputs) {
        std::cout << "Running " << name << "\n";
        return func.invoke(std::move(inputs));
      });
}

auto create_pipeline(int seed, int element) {
  return make_pipeline<int, int>(
      {"Create", "Shuffle", "Sort", "Get"},
      std::make_tuple(create_vector, create_shuffle(seed), sort_vector,
                      get_element(element)));
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
      .output<int>(("final_sum"));
}

template <typename Graph>
void execute_graph_with_threads(Graph g) {
  int size = 100'000;
  int max_threads = 2;

  for(int num_threads = 1; num_threads <= max_threads; num_threads++) {
    TaskSystem task_system(num_threads);

    auto decorated_graph = g
                               // .decorate(timing_decorator)
                               .decorate(print_decorator);

    auto t0 = std::chrono::steady_clock::now();
    std::cout << "--------- " << num_threads << " THREADS --------\n";
    uint64_t result = execute_graph(decorated_graph, task_system, size);
    auto t1 = std::chrono::steady_clock::now();
    std::cout << "result is " << result << " after "
              << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0)
                     .count()
              << "us\n";
  }
}

void take_ref(const Sentinal& sen) {
  std::cout << "Ref " << sen.copies << " " << sen.moves << "\n";
}

void take_val(Sentinal sen) {
  std::cout << "Val " << sen.copies << " " << sen.moves << "\n";
}

void test_ref_graph() {
  auto g = make_graph<Sentinal>({"in"})
               .add(take_ref, "ref", {".in"})
               .add(take_val, "val", {".in"})
               .output<Sentinal>((".in"));

  TaskSystem task_system(1);
  Sentinal result = execute_graph(g, task_system, Sentinal());

  std::cout << "Res " << result.copies << " " << result.moves << "\n";
}

} // namespace

int main() {
  execute_graph_with_threads(create_graph());
  test_ref_graph();
  any_function_test();
  // stress_test();

  return 0;
}