#include "any_function.h"

#include "graph.h"
#include "test.h"
#include "executor.h"
#include "tasks.h"

#include <iostream>

using namespace anyf;

int create_3() {
  return 3;
}

int double_value(int x) {
  return x * 2;
}

int sum(int x, int y) {
  return x  + y;
}

int main() {
  any_function_test();
  TaskSystem task_system;

  // std::this_thread::sleep_for(std::chrono::milliseconds(10000));

  auto g = make_graph<int>({"input"})
        .add(create_3, "gen3")
        .add(double_value, "double", {"input"})
        .add(sum, "sum1", {"gen3", "gen3"})
        .add(sum, "sum2", {"sum1", "double"})
        .output<int>("sum2");

  auto result = execute_graph(g, task_system, 5);

  std::cout << "result is " << result << "\n";

  return 0;
}