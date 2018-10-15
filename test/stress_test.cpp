#include "graph_execution.h"
#include "executor/tbb_executor.h"

#include <chrono>
#include <thread>

#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

using namespace anyf;

namespace {
int create_int() { return 1; }

int sum(int x, int y) { return x + y; }

function_graph<int> create_graph(int depth) {
  auto g = make_graph();

  std::string prefix("node");

  int num_inputs = 1 << depth;
  for(int i = 0; i < num_inputs; i++) {
    g.add(create_int, prefix + std::to_string(depth) + "_" + std::to_string(i));
  }

  for(int layer = depth - 1; layer >= 0; layer--) {
    int nodes_on_layer = 1 << layer;
    for(int i = 0; i < nodes_on_layer; i++) {
      g.add(sum, prefix + std::to_string(layer) + "_" + std::to_string(i),
            {prefix + std::to_string(layer + 1) + "_" + std::to_string((i * 2)),
             prefix + std::to_string(layer + 1) + "_" +
                 std::to_string((i * 2) + 1)});
    }
  }

  return g.output<int>(prefix + "0_0");
}
} // namespace

BOOST_AUTO_TEST_CASE(stress_test) {
  const int depth = 14;
  const int num_executions = 100;
  tbb_executor executor;

  std::cout << "Thread count: " << std::thread::hardware_concurrency() << '\n';
  std::cout << "Creating graph of size " << ((1 << depth) * 2 - 1) << "\n";

  auto t0 = std::chrono::steady_clock::now();
  auto g = create_graph(depth);
  auto t1 = std::chrono::steady_clock::now();

  std::cout
      << "Creating graph took "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
      << "ms\n";

  t0 = std::chrono::steady_clock::now();
  int result;
  for(int i = 0; i < num_executions; i++) {
    result = execute_graph(g, executor);
  }
  t1 = std::chrono::steady_clock::now();

  std::cout
      << "Result is " << result << ", " << num_executions << " executions took "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
      << "ms\n";
}