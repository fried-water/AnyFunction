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

#include <boost/test/unit_test.hpp>

using namespace anyf;

namespace {
std::vector<int> create_vector(int size) {
  std::mt19937 rng;
  std::uniform_int_distribution<> dist(0, 100'000);
  std::vector<int> result;
  result.reserve(size);
  for(int i = 0; i < size; i++) {
    result.push_back(dist(rng));
  }

  return result;
}

auto create_shuffle(int seed) {
  return [seed](std::vector<int> vec) {
    std::shuffle(vec.begin(), vec.end(), std::mt19937(seed));
    return vec;
  };
}

int64_t accumulate(const std::vector<int>& v) {
  return std::accumulate(v.begin(), v.end(), int64_t(0));
}

std::vector<int> sort_vector(std::vector<int> vec) {
  std::sort(vec.begin(), vec.end());

  return vec;
}

int64_t sum(int64_t x, int64_t y) { return x + y; }

auto create_pipeline(int seed) {
  auto create = Delayed(create_vector);
  auto shuffle = Delayed(create_shuffle(seed));
  auto sort = Delayed(sort_vector);
  auto acc = Delayed(accumulate);

  auto [g, size] = make_graph<int>();
  return FunctionGraph(std::move(g), acc(sort(shuffle(create(size)))));
}

auto create_graph() {
  auto [g, size] = make_graph<int>();

  std::array<Edge<int64_t>, 8> ps = {create_pipeline(0)(size), create_pipeline(1)(size),
                                     create_pipeline(2)(size), create_pipeline(3)(size),
                                     create_pipeline(4)(size), create_pipeline(5)(size),
                                     create_pipeline(6)(size), create_pipeline(7)(size)};

  auto del_sum = Delayed(sum);
  return FunctionGraph(std::move(g),
                       del_sum(del_sum(del_sum(ps[0], ps[1]), del_sum(ps[2], ps[3])),
                               del_sum(del_sum(ps[4], ps[5]), del_sum(ps[6], ps[7]))));
}

template <typename Executor, typename Graph>
void execute_graph_with_threads(Graph g) {
  const int size = 500'000;
  const int max_threads = 8;

  for(int num_threads = 1; num_threads <= max_threads; num_threads++) {
    Executor executor(num_threads);

    const auto t0 = std::chrono::steady_clock::now();
    const int64_t result = execute_graph(g, executor, size);
    const auto t1 = std::chrono::steady_clock::now();
    std::cout << num_threads << " THREADS: result is " << result << " after "
              << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() << "us\n";
  }
}

} // namespace

BOOST_AUTO_TEST_CASE(example_graph_tbb) {
  std::cout << "\nExecuting graph with TBB\n\n";
  execute_graph_with_threads<TBBExecutor>(create_graph());
}

BOOST_AUTO_TEST_CASE(example_graph_task) {
  std::cout << "\nExecuting graph with custom task system\n\n";
  execute_graph_with_threads<TaskExecutor>(create_graph());
}

BOOST_AUTO_TEST_CASE(example_graph_seq) {
  std::cout << "\nExecuting graph Sequentially\n\n";
  execute_graph_with_threads<SequentialExecutor>(create_graph());
}

BOOST_AUTO_TEST_CASE(test_graph_input_sentinal) {
  auto take = Delayed([](Sentinal sent) { return sent; });

  auto take_ref = Delayed([](const Sentinal& sent) {
    BOOST_CHECK_EQUAL(0, sent.copies);
    return sent;
  });

  auto [cg, s1, s2, s3] = make_graph<Sentinal, Sentinal, Sentinal>();

  FunctionGraph g(std::move(cg), take(take(take(s1))), take(s2), s2, take_ref(s3), s3);

  SequentialExecutor executor;
  auto [r1, r2, r3, r4, r5] = execute_graph(g, executor, Sentinal{}, Sentinal{}, Sentinal{});
  BOOST_CHECK_EQUAL(0, r1.copies); // Move since s1 no other inputs
  BOOST_CHECK_EQUAL(1, r2.copies); // s2 has two outputs first is copied
  BOOST_CHECK_EQUAL(0, r3.copies); //   second is moved
  BOOST_CHECK_EQUAL(1, r4.copies); // s3 must be copied through take_ref
  BOOST_CHECK_EQUAL(1, r5.copies); // s3 cannot be moved since taken by ref elsewhere
}
