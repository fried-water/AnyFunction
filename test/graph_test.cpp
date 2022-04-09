#include "graph.h"

#include <algorithm>
#include <iostream>

#include <boost/test/unit_test.hpp>

using namespace anyf;
using namespace anyf::graph;

int identity(int x) { return x; }
int cidentity(const int& x) { return x; }
int by2(const int& x) { return x * 2; }
auto sum(int x, int y) { return x + y; }

bool compare_edges_to_nodes(const std::vector<NodeEdge>& expected, const Node& actual) {
  return std::equal(expected.begin(), expected.end(), actual.outputs.begin(), actual.outputs.end());
}

bool compare_nodes(const Node& expected, const Node& actual) {
  return expected.outputs == actual.outputs;
}

BOOST_AUTO_TEST_CASE(simple_graph) {
  auto [cg, in1, in2] = make_graph<int, int>();
  FunctionGraph g(std::move(cg), Delayed(sum)(Delayed(cidentity)(in1), in2));

  std::vector<std::vector<NodeEdge>> expected_edges{
      {{0, {1, 0}, PassBy::ref}, {1, {2, 1}, PassBy::copy}},
      {{0, {2, 0}, PassBy::copy}},
      {{0, {3, 0}, PassBy::copy}},
      {}};

  BOOST_CHECK(std::equal(expected_edges.begin(), expected_edges.end(), g.nodes().begin(),
                         g.nodes().end(), compare_edges_to_nodes));
}

BOOST_AUTO_TEST_CASE(inside_empty_graph) {
  auto [inner_cg, in3, in4] = make_graph<int, int>();
  FunctionGraph inner_g(std::move(inner_cg), Delayed(identity)(Delayed(by2)(
                                                 Delayed(sum)(Delayed(cidentity)(in3), in4))));

  auto [cg, in1, in2] = make_graph<int, int>();
  FunctionGraph g(std::move(cg), inner_g(in1, in2));

  BOOST_CHECK(std::equal(inner_g.nodes().begin(), inner_g.nodes().end(), g.nodes().begin(),
                         g.nodes().end(), compare_nodes));
}

BOOST_AUTO_TEST_CASE(inner_graph) {
  auto [cg, in1, in2] = make_graph<int, int>();

  Edge<int> id = Delayed(identity)(in1);
  Edge<int> cid = Delayed(cidentity)(in2);

  auto [inner_cg, in3, in4] = make_graph<int, int>();
  FunctionGraph inner_g(std::move(inner_cg), Delayed(sum)(Delayed(cidentity)(in3), in4));

  FunctionGraph g(std::move(cg), inner_g(id, cid));

  std::vector<std::vector<NodeEdge>> expected_edges{
      {{0, {1, 0}, PassBy::copy}, {1, {2, 0}, PassBy::ref}},
      {{0, {3, 0}, PassBy::ref}},
      {{0, {4, 1}, PassBy::copy}},
      {{0, {4, 0}, PassBy::copy}},
      {{0, {5, 0}, PassBy::copy}},
      {}};

  BOOST_CHECK(std::equal(expected_edges.begin(), expected_edges.end(), g.nodes().begin(),
                         g.nodes().end(), compare_edges_to_nodes));
}
