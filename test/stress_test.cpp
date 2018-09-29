#include "executor.h"
#include "tasks.h"
#include "test.h"

#include <chrono>

using namespace anyf;

namespace {
int create_int() { return 1; }

int sum(int x, int y) {
  return x + y;
}

function_graph<int> create_graph(int depth) {
  auto g = make_graph();

  int num_inputs = 1 << depth;
  for(int i = 0; i < num_inputs; i++) {
    g.add(create_int, std::to_string(depth) + ":" + std::to_string(i));
  }

  for(int layer = depth - 1; layer >= 0; layer--) {
    int nodes_on_layer = 1 << layer;
    for(int i = 0; i < nodes_on_layer; i++) {
      g.add(sum, std::to_string(layer) + ":" + std::to_string(i),
            {std::to_string(layer + 1) + ":" + std::to_string((i * 2)),
             std::to_string(layer + 1) + ":" + std::to_string((i * 2) + 1)});
    }
  }

  return g.output<int>("0:0");
}
} // namespace

void stress_test() {
  const int depth = 14;
  const int num_executions = 100;
  TaskSystem task_system;

  std::cout << "Thread count: " << std::thread::hardware_concurrency()
            << '\n';
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
    result = execute_graph(g, task_system);
  }
  t1 = std::chrono::steady_clock::now();

  std::cout
      << "Result is " << result << ", " << num_executions << " executions took "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
      << "ms\n";
}