#include "higher_order.h"

#include "executor/sequential_executor.h"
#include "graph_execution.h"
#include "sentinal.h"

#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

using namespace anyf;

int multiply(int x, const int& y, std::string str) { return x * (y + str.size()); }

// BOOST_AUTO_TEST_CASE(test_map) {
//   auto [cg, vec, cint, str] = make_graph<std::vector<int>, int, std::string>();
//   FunctionGraph g(std::move(cg), map(multiply, vec, cint, str));

//   std::vector<int> input{1, 2, 3};
//   TBBExecutor executor;
//   auto res = execute_graph(g, executor, std::move(input), 7, std::string{"abc"});

//   std::vector<int> expected{10, 20, 30};
//   BOOST_CHECK(res == expected);
// }

// BOOST_AUTO_TEST_CASE(test_accumulate) {
//   auto sum = [](std::string x, int y, const char& base) { return x + static_cast<char>(base + y); };

//   auto [cg, vec, str, base] = make_graph<std::vector<int>, std::string, char>();
//   FunctionGraph g(std::move(cg), accumulate(sum, vec, str, base));

//   std::vector<int> input{1, 2, 3};
//   TBBExecutor executor;
//   auto res = execute_graph(g, executor, std::move(input), std::string{}, 'c');

//   BOOST_CHECK("def" == res);
// }

// BOOST_AUTO_TEST_CASE(test_map_graph) {
//   auto func = [](int x, const int& y, std::string str) -> int { return x * (y + str.size()); };
//   auto [map_cg, x, y, str] = make_graph<int, int, std::string>();
//   FunctionGraph multiply_g(std::move(map_cg), Delayed(func)(x, y, str));

//   auto [cg, vec, cint, str2] = make_graph<std::vector<int>, int, std::string>();
//   FunctionGraph g(std::move(cg), map(multiply_g, vec, cint, str2));

//   std::vector<int> input{1, 2, 3};
//   TBBExecutor executor;
//   auto res = execute_graph(g, executor, std::move(input), 7, std::string{"abc"});

//   std::vector<int> expected{10, 20, 30};

//   BOOST_CHECK(res == expected);
// }


BOOST_AUTO_TEST_CASE(test_map_graph_sentinal) {
  auto func = [](Sentinal x, const Sentinal& y, Sentinal z) {
    std::cout << "X copies: " << x.copies << " moves: " << x.moves << "\n";
    std::cout << "Y copies: " << y.copies << " moves: " << y.moves << "\n";
    std::cout << "Z copies: " << z.copies << " moves: " << z.moves << "\n";
    return x;
  };
  auto [map_cg, x, y, z] = make_graph<Sentinal, Sentinal, Sentinal>();
  FunctionGraph multiply_g(std::move(map_cg), Delayed(func)(x, y, z));

  auto [cg, vec, cy, cz] = make_graph<std::vector<Sentinal>, Sentinal, Sentinal>();
  FunctionGraph g(std::move(cg), map(multiply_g, vec, cy, cz));

  std::vector<Sentinal> input = util::make_std_vector<Sentinal>(Sentinal{});
  SequentialExecutor executor;
  auto res = execute_graph(g, executor, std::move(input), Sentinal{}, Sentinal{});

  for(const auto& sent : res) {
    BOOST_CHECK_EQUAL(0, sent.copies);
  }
}
