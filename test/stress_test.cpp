#include "anyf/executor/task_executor.h"
#include "anyf/graph_execution.h"

#include <boost/test/unit_test.hpp>

#include <chrono>
#include <thread>
#include <unordered_map>

using namespace anyf;

namespace {
int identity(int x) { return x; }

int sum(const int& x, int y) { return x + y; }

auto create_graph(int depth) {
  std::unordered_map<std::pair<int, int>, DelayedEdge<int>, knot::Hash> edges;

  auto Delayed_copy = Delayed(identity);
  auto Delayed_sum = Delayed(sum);

  auto [g, input] = make_graph<int>();
  int num_inputs = 1 << depth;
  for(int i = 0; i < num_inputs; i++) {
    edges.emplace(std::pair(depth, i), Delayed_copy(input));
  }

  for(int layer = depth - 1; layer >= 0; layer--) {
    int nodes_on_layer = 1 << layer;
    for(int i = 0; i < nodes_on_layer; i++) {
      edges.emplace(std::pair(layer, i),
                    Delayed_sum(edges.at(std::pair(layer + 1, i * 2)), edges.at(std::pair(layer + 1, i * 2 + 1))));
    }
  }

  return finalize(std::move(g), edges.at(std::pair(0, 0)));
}

} // namespace

BOOST_AUTO_TEST_CASE(stress_test, *boost::unit_test::disabled()) {
  const int depth = 12;
  const int num_executions = 100;
  auto executor = make_task_executor();

  fmt::print("Thread count: {}\n", std::thread::hardware_concurrency());
  fmt::print("Creating graph of size {}\n", ((1 << depth) * 2 - 1));

  auto t0 = std::chrono::steady_clock::now();
  auto g = create_graph(depth);

  fmt::print("Creating graph took {}ms\n",
             std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count());

  t0 = std::chrono::steady_clock::now();
  int result = -1;
  for(int i = 0; i < num_executions; i++) {
    result = execute_graph(g, executor, 1);
  }

  executor.wait();

  fmt::print("Result is {}, {} executions took {}ms\n",
             result,
             num_executions,
             std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count());
}
