#include "graph.h"

#include <algorithm>
#include <iostream>

#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

using namespace anyf;
using namespace anyf::graph;

int identity(int x) { return x; }
int cidentity(const int& x) { return x; }
int by2(const int& x) { return x * 2; }
auto sum(int x, int y) { return x + y; }

bool compare_nodes(const Node& expected, const Node& actual) {
  return expected.outputs == actual.outputs;
}

BOOST_AUTO_TEST_CASE(simple_graph) {
  auto [cg, in1, in2] = make_graph<int, int>();
  FunctionGraph g(std::move(cg), Delayed(sum)(Delayed(cidentity)(in1), in2));

  std::vector<Node> expected_nodes{Node{}, Node{AnyFunction(identity)}, Node{AnyFunction(sum)},
                                   Node{}};

  expected_nodes[0].outputs = {{0, {1, 0}, PassBy::ref}, {1, {2, 1}, PassBy::copy}};
  expected_nodes[1].outputs = {{0, {2, 0}, PassBy::copy}};
  expected_nodes[2].outputs = {{0, {3, 0}, PassBy::copy}};

  BOOST_CHECK(std::equal(expected_nodes.begin(), expected_nodes.end(), g.nodes().begin(),
                         g.nodes().end(), compare_nodes));
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

  std::vector<Node> expected_nodes{Node{},
                                   Node{AnyFunction(identity)},
                                   Node{AnyFunction(cidentity)},
                                   Node{AnyFunction(cidentity)},
                                   Node{AnyFunction(sum)},
                                   Node{}};

  expected_nodes[0].outputs = {{0, {1, 0}, PassBy::copy}, {1, {2, 0}, PassBy::ref}};
  expected_nodes[1].outputs = {{0, {3, 0}, PassBy::ref}};
  expected_nodes[2].outputs = {{0, {4, 1}, PassBy::copy}};
  expected_nodes[3].outputs = {{0, {4, 0}, PassBy::copy}};
  expected_nodes[4].outputs = {{0, {5, 0}, PassBy::copy}};

  BOOST_CHECK(std::equal(expected_nodes.begin(), expected_nodes.end(), g.nodes().begin(),
                         g.nodes().end(), compare_nodes));
}
