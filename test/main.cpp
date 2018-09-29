#include "executor.h"
#include "tasks.h"
#include "test.h"

#include <iostream>

using namespace anyf;

namespace {
int create_3() { return 3; }

int double_value(int x) { return x * 2; }

int sum(int x, int y) { return x + y; }

auto create_simple_graph() {
  return make_graph<int>({"input"})
               .add(create_3, "gen3")
               .add(double_value, "double", {"input"})
               .add(sum, "sum1", {"gen3", "gen3"})
               .add(sum, "sum2", {"sum1", "double"})
               .output<int>("sum2");
}

void execute_simple_graph() {
  TaskSystem task_system;
  std::cout << "result is " << execute_graph(create_simple_graph(), task_system, 5) << "\n";
}

} // namespace

int main() {
  // execute_simple_graph();
  // any_function_test();
  stress_test();

  return 0;
}