#include "anyf/executor/task_executor.h"
#include "anyf/graph_execution.h"

#include <boost/test/unit_test.hpp>

#include <chrono>
#include <thread>
#include <unordered_map>

namespace anyf {

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
  auto ex = make_task_executor();

  fmt::print("Thread count: {}\n", std::thread::hardware_concurrency());
  fmt::print("Creating graph of size {}\n", ((1 << depth) * 2 - 1));

  auto t0 = std::chrono::steady_clock::now();
  auto g = create_graph(depth);

  fmt::print("Creating graph took {}ms\n",
             std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count());

  t0 = std::chrono::steady_clock::now();
  int result = -1;
  for(int i = 0; i < num_executions; i++) {
    result = execute_graph(g, ex, 1);
  }

  fmt::print("Result is {}, {} executions took {}ms\n",
             result,
             num_executions,
             std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count());
}

BOOST_AUTO_TEST_CASE(stress_test_functional) {
  const int num_executions = 100;
  auto ex = make_task_executor();

  auto [cg, inputs] = make_graph(make_type_properties(TypeList<FunctionGraph, int, int>{}));
  const auto g = *std::move(cg).finalize(*cg.add_functional(
    {{type_id<int>(), true}, {type_id<int>(), false}}, {type_id<int>()}, inputs[0], {inputs[1], inputs[2]}));

  for(int i = 0; i < num_executions; i++) {
    std::vector<Future> owned_inputs;
    owned_inputs.push_back(Future(ex, Any(make_graph(AnyFunction([i](int x, const int& y) { return x + y + i; })))));
    owned_inputs.push_back(Future(ex, Any(5)));
    owned_inputs.push_back(Future(ex, Any(7)));

    auto outputs = execute_graph(g, ex, std::move(owned_inputs), {});
    BOOST_CHECK_EQUAL(i + 12, any_cast<int>(std::move(outputs.front()).wait()));
  }
}

BOOST_AUTO_TEST_CASE(stress_test_if) {
  const int num_executions = 100;
  auto ex = make_task_executor();

  const auto identity = [](int x) { return x; };
  const auto add1 = [](int x) { return x + 1; };

  auto [cg, inputs] = make_graph(make_type_properties(TypeList<bool, int>{}));
  const auto g =
    *std::move(cg).finalize(*cg.add_if(make_graph(AnyFunction(identity)), make_graph(AnyFunction(add1)), inputs));

  for(int i = 0; i < num_executions; i++) {
    std::vector<Future> owned_inputs;
    owned_inputs.push_back(Future(ex, Any(i % 2 == 0)));
    owned_inputs.push_back(Future(ex, Any(0)));

    auto outputs = execute_graph(g, ex, std::move(owned_inputs), {});
    BOOST_CHECK_EQUAL(i % 2, any_cast<int>(std::move(outputs.front()).wait()));
  }
}

BOOST_AUTO_TEST_CASE(stress_test_select) {
  const int num_executions = 100;
  auto ex = make_task_executor();

  auto [cg, inputs] = make_graph(make_type_properties(TypeList<bool, int, int>{}));
  const auto g = *std::move(cg).finalize(*cg.add_select(inputs[0], {inputs[1]}, {inputs[2]}));

  for(int i = 0; i < num_executions; i++) {
    std::vector<Future> owned_inputs;
    owned_inputs.push_back(Future(ex, Any(i % 2 == 0)));
    owned_inputs.push_back(Future(ex, Any(0)));
    owned_inputs.push_back(Future(ex, Any(1)));

    auto outputs = execute_graph(g, ex, std::move(owned_inputs), {});
    BOOST_CHECK_EQUAL(i % 2, any_cast<int>(std::move(outputs.front()).wait()));
  }
}

BOOST_AUTO_TEST_CASE(stress_test_while) {
  const int num_executions = 100;
  auto ex = make_task_executor();

  const auto body = [](int x, const int& limit) { return std::tuple(x + 1 < limit, x + 1); };

  auto [cg, inputs] = make_graph(make_type_properties(TypeList<bool, int, const int&>{}));
  const auto g = *std::move(cg).finalize(*cg.add_while(make_graph(AnyFunction(body)), inputs));

  for(int i = 0; i < num_executions; i++) {
    const int limit = i % 10;

    std::vector<Future> owned_inputs;
    owned_inputs.push_back(Future(ex, Any(0 < limit)));
    owned_inputs.push_back(Future(ex, Any(0)));

    std::vector<BorrowedFuture> borrowed_inputs;
    borrowed_inputs.push_back(borrow(Future(ex, Any(limit))).first);

    auto outputs = execute_graph(g, ex, std::move(owned_inputs), borrowed_inputs);
    BOOST_CHECK_EQUAL(limit, any_cast<int>(std::move(outputs.front()).wait()));
  }
}

} // namespace anyf
