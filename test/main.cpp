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
      .output<int>("final_sum");
}

template <typename Graph>
void execute_graph_with_threads(Graph g) {
  int size = 100'000;
  int max_threads = 4;

  for(int num_threads = 1; num_threads <= max_threads; num_threads++) {
    TaskSystem task_system(num_threads);

    auto decorated_graph = g;
    // .decorate(timing_decorator)
    // .decorate(print_decorator);

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

struct SentinalDestructor {
  bool valid = true;

  SentinalDestructor() {}

  ~SentinalDestructor() {
    if(valid) {
      std::cout << "Destruct--------------------------------\n";
    }
  }

  SentinalDestructor(const SentinalDestructor& other) : valid(other.valid) {}

  SentinalDestructor(SentinalDestructor&& other) : valid(other.valid) {
    other.valid = false;
  }
};

auto take(int i) {
  return [i](Sentinal sent) {
    std::cout << "Take" << i << " " << sent.copies << " " << sent.moves << "\n";
  };
}

void take_ref(const Sentinal& sent) {
  std::cout << "Ref "
            << " " << sent.copies << " " << sent.moves << "\n";
}

void test_graph() {
  auto g = make_graph<Sentinal, Sentinal>({"in1", "in2"})
               .add(take(1), "val", {".in1"}, {pass_by::copy})
               .add(take(2), "val2", {".in1"}, {pass_by::move})
               .add(take(3), "val3", {".in2"}, {pass_by::move})
               .add(take_ref, "ref", {".in2"}, {pass_by::ref})
               .output<Sentinal>(".in2");

  TaskSystem task_system(1);
  Sentinal x = execute_graph(g, task_system, Sentinal(), Sentinal());
  std::cout << "Result "
            << " " << x.copies << " " << x.moves << "\n";
}

} // namespace

int main() {
  execute_graph_with_threads(create_graph());
  test_graph();
  any_function_test();
  // stress_test();

  return 0;
}