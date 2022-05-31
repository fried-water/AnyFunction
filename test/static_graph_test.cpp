#include "anyf/static_graph.h"

#include <boost/test/unit_test.hpp>

#include <algorithm>

using namespace anyf;

int identity(int x) { return x; }
int cidentity(const int& x) { return x; }
int by2(const int& x) { return x * 2; }
auto sum(int x, int y) { return x + y; }

bool compare_edges_to_nodes(const std::vector<Edge>& expected, const Node& actual) {
  return std::equal(expected.begin(), expected.end(), actual.outputs.begin(), actual.outputs.end());
}

bool compare_nodes(const Node& expected, const Node& actual) { return expected.outputs == actual.outputs; }

BOOST_AUTO_TEST_CASE(simple_graph) {
  auto [cg, in1, in2] = make_graph<int, int>();
  const auto g = finalize(std::move(cg), Delayed(sum)(Delayed(cidentity)(in1), in2));

  const std::vector<std::vector<Edge>> expected_edges{{{0, {1, 0}}, {1, {2, 1}}}, {{0, {2, 0}}}, {{0, {3, 0}}}, {}};

  BOOST_CHECK(std::equal(expected_edges.begin(), expected_edges.end(), g.begin(), g.end(), compare_edges_to_nodes));
}

BOOST_AUTO_TEST_CASE(inside_empty_graph) {
  auto [inner_cg, in3, in4] = make_graph<int, int>();
  const auto inner_g =
    finalize(std::move(inner_cg), Delayed(identity)(Delayed(by2)(Delayed(sum)(Delayed(cidentity)(in3), in4))));

  auto [cg, in1, in2] = make_graph<int, int>();
  const auto g = finalize(std::move(cg), inner_g(in1, in2));

  BOOST_CHECK(std::equal(inner_g.begin(), inner_g.end(), g.begin(), g.end(), compare_nodes));
}

BOOST_AUTO_TEST_CASE(inner_graph) {
  auto [cg, in1, in2] = make_graph<int, int>();

  DelayedEdge<int> id = Delayed(identity)(in1);
  DelayedEdge<int> cid = Delayed(cidentity)(in2);

  auto [inner_cg, in3, in4] = make_graph<int, int>();
  const auto inner_g = finalize(std::move(inner_cg), Delayed(sum)(Delayed(cidentity)(in3), in4));

  const auto g = finalize(std::move(cg), inner_g(id, cid));

  const std::vector<std::vector<Edge>> expected_edges{
    {{0, {1, 0}}, {1, {2, 0}}}, {{0, {3, 0}}}, {{0, {4, 1}}}, {{0, {4, 0}}}, {{0, {5, 0}}}, {}};

  BOOST_CHECK(std::equal(expected_edges.begin(), expected_edges.end(), g.begin(), g.end(), compare_edges_to_nodes));
}
