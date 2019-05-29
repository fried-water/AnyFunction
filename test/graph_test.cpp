#include "graph_new.h"

#include <algorithm>
#include <iostream>

#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

using namespace anyf;
using namespace anyf::graph;

int identity(int x) { return x; }
int cidentity(const int& x) { return x; }
auto sum(int x, int y) { return x + y; }

bool compare_nodes(const Node& expected, const Node& actual) {
  return expected.types() == actual.types() &&
    expected.outputs == actual.outputs;
}

BOOST_AUTO_TEST_CASE(testing) {
  auto [con_g, in1, in2] = make_graph<int, int>();

  Edge<int> id = fg(identity)(in1);
  Edge<int> cid = fg(cidentity)(in2);
}

BOOST_AUTO_TEST_CASE(simple_graph) {
  // auto g = make_graph<int>({"in1", "in2"})
  //   .add(cidentity, "identity", {".in1"})
  //   .add(sum, "sum", {"identity", ".in2"})
  //   .outputs<int>({"sum"});

  // std::vector<Node> expected_nodes{
  //   Node{{{make_type<int>(), make_type<int>()}}, "", {"in1", "in2"}},
  //   Node{{make_any_function(cidentity)}, "identity", {}},
  //   Node{{make_any_function(sum)}, "sum", {}},
  //   Node{{{make_type<int>()}}}
  // };

  auto [con_g, in1, in2] = make_graph<int, int>();
  auto g = std::move(con_g).outputs<int>(fg(sum)(fg(cidentity)(in1), in2));

  std::vector<Node> expected_nodes{
    Node{Source{{make_type<int>(), make_type<int>()}}},
    Node{FuncNode{make_any_function(identity)}},
    Node{FuncNode{make_any_function(sum)}},
    Node{Sink{{make_type<int>()}}}
  };

  expected_nodes[0].outputs = { {{0, 0}, {1, 0}, PassBy::ref}, {{0, 1}, {2, 1}, PassBy::copy} };
  expected_nodes[1].outputs = { {{1, 0}, {2, 0}, PassBy::copy} };
  expected_nodes[2].outputs = { {{2, 0}, {3, 0}, PassBy::copy} };

  BOOST_CHECK(std::equal(
    expected_nodes.begin(), expected_nodes.end(), g.nodes().begin(), g.nodes().end(), compare_nodes));
}

