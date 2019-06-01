#include "higher_order.h"

#include "executor/tbb_executor.h"
#include "graph_execution.h"

#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

using namespace anyf;

int multiply(int x, const int& y, std::string str) { return x * (y + str.size()); }

BOOST_AUTO_TEST_CASE(test_map) {
  auto [cong, vec, cint, str] = make_graph<std::vector<int>, int, std::string>();
  auto g = std::move(cong).outputs(map(multiply, vec, cint, str));

  std::vector<int> input{1, 2, 3};
  tbb_executor executor;
  auto res = execute_graph(g, executor, std::move(input), 7, std::string{"abc"});

  std::vector<int> expected{10, 20, 30};
  BOOST_CHECK(res == expected);
}

BOOST_AUTO_TEST_CASE(test_accumulate) {
  auto sum = [](std::string x, int y, const char& base) { return x + static_cast<char>(base + y); };

  auto [cong, vec, str, base] = make_graph<std::vector<int>, std::string, char>();
  auto g = std::move(cong).outputs(accumulate(sum, vec, str, base));

  std::vector<int> input{1, 2, 3};
  tbb_executor executor;
  auto res = execute_graph(g, executor, std::move(input), std::string{}, 'c');

  BOOST_CHECK("def" == res);
}
