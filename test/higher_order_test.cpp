#include "higher_order.h"

#include "executor/tbb_executor.h"
#include "graph_execution.h"

#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

using namespace anyf;

int multiply(int x, const int& y) { return x * y; }
int by2(int x) { return x * 2; }

BOOST_AUTO_TEST_CASE(test_map) {
  auto [cong, vec, cint] = make_graph<std::vector<int>, int>();
  auto g = std::move(cong).outputs<std::vector<int>>(map(multiply, vec, cint));

  // auto [cong, vec] = make_graph<std::vector<int>>();
  // auto g = std::move(cong).outputs<std::vector<int>>(map(by2, vec));

  std::vector<int> input{1, 2, 3};
  tbb_executor executor;
  auto res = execute_graph(g, executor, std::move(input), 7);

  std::vector<int> expected_7{7, 14, 21};
  std::vector<int> expected_2{2, 4, 6};
  BOOST_CHECK(res == expected_7);
}
