#include "executor/tbb_executor.h"
#include "graph_execution.h"

#include <chrono>
#include <thread>
#include <unordered_map>

#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

using namespace anyf;

namespace {
int identity(int x) { return x; }

int sum(const int& x, int y) { return x + y; }

struct IntPairHash {
  std::size_t operator()(std::pair<int, int> p) const {
    return (((std::size_t)p.first) << 32) + ((std::size_t)p.second);
  }
};

FunctionGraph<std::tuple<int>, std::tuple<int>> create_graph(int depth) {
  std::unordered_map<std::pair<int, int>, Edge<int>, IntPairHash> edges;

  auto fg_copy = fg(identity);
  auto fg_sum = fg(sum);

  auto [g, input] = make_graph<int>();
  int num_inputs = 1 << depth;
  for(int i = 0; i < num_inputs; i++) {
    edges.emplace(std::pair(depth, i), fg_copy(input));
  }

  for(int layer = depth - 1; layer >= 0; layer--) {
    int nodes_on_layer = 1 << layer;
    for(int i = 0; i < nodes_on_layer; i++) {
      edges.emplace(std::pair(layer, i), fg_sum(edges.at(std::pair(layer + 1, i * 2)),
                                                edges.at(std::pair(layer + 1, i * 2 + 1))));
    }
  }

  return FunctionGraph(std::move(g), edges.at(std::pair(0, 0)));
}
} // namespace

BOOST_AUTO_TEST_CASE(stress_test) {
  const int depth = 14;
  const int num_executions = 100;
  TBBExecutor executor;

  std::cout << "Thread count: " << std::thread::hardware_concurrency() << '\n';
  std::cout << "Creating graph of size " << ((1 << depth) * 2 - 1) << "\n";

  auto t0 = std::chrono::steady_clock::now();
  auto g = create_graph(depth);
  auto t1 = std::chrono::steady_clock::now();

  std::cout << "Creating graph took "
            << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << "ms\n";

  t0 = std::chrono::steady_clock::now();
  int result;
  for(int i = 0; i < num_executions; i++) {
    result = execute_graph(g, executor, 1);
  }
  t1 = std::chrono::steady_clock::now();

  std::cout << "Result is " << result << ", " << num_executions << " executions took "
            << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << "ms\n";
}