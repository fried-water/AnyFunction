#include "graph.h"

#include <algorithm>
#include <iostream>

#include <boost/test/unit_test.hpp>

using namespace anyf;
using namespace anyf::graph;

namespace {

int identity(int x) { return x; }
int cidentity(const int& x) { return x; }
int by2(const int& x) { return x * 2; }
auto sum(int x, int y) { return x + y; }

bool compare_edges_to_nodes(const std::vector<graph::Edge>& expected, const Node& actual) {
  return std::equal(expected.begin(), expected.end(), actual.outputs.begin(), actual.outputs.end());
}

bool compare_node_outputs(const Node& expected, const Node& actual) {
  return expected.outputs == actual.outputs;
}

} // namespace

BOOST_AUTO_TEST_CASE(simple_graph_construction) {
  auto [cg, inputs] = make_graph(make_types(TL<int, int>{}));
  const auto id_outputs = cg.add(AnyFunction(cidentity), std::array{inputs[0]});
  const auto sum_outputs = cg.add(AnyFunction(sum), std::array{id_outputs[0], inputs[1]});
  const FunctionGraph g = finalize(std::move(cg), sum_outputs);

  const std::vector<std::vector<graph::Edge>> expected_edges{
      {{0, {1, 0}}, {1, {2, 1}}}, {{0, {2, 0}}}, {{0, {3, 0}}}, {}};

  BOOST_CHECK((std::vector<graph::Term>{{0, 0}, {0, 1}}) == inputs);
  BOOST_CHECK((std::vector<graph::Term>{{1, 0}}) == id_outputs);
  BOOST_CHECK((std::vector<graph::Term>{{2, 0}}) == sum_outputs);

  BOOST_CHECK(std::equal(expected_edges.begin(), expected_edges.end(), g.begin(), g.end(),
                         compare_edges_to_nodes));

  BOOST_CHECK(make_types(TL<int, int>{}) == std::get<std::vector<Type>>(g.front().func));
  BOOST_CHECK(make_types(TL<int>{}) == std::get<std::vector<Type>>(g.back().func));
}

BOOST_AUTO_TEST_CASE(simple_inner_graph) {

  auto [inner_cg, inner_inputs] = make_graph(make_types(TL<int, int>{}));
  const auto inner_id_outputs = inner_cg.add(AnyFunction(cidentity), std::array{inner_inputs[0]});
  const auto inner_sum_outputs =
      inner_cg.add(AnyFunction(sum), std::array{inner_id_outputs[0], inner_inputs[1]});
  const FunctionGraph inner_g = finalize(std::move(inner_cg), inner_sum_outputs);

  auto [cg, inputs] = make_graph(make_types(TL<int, int>{}));
  const auto id_outputs = cg.add(AnyFunction(identity), std::array{inputs[0]});
  const auto g_outputs = cg.add(inner_g, std::array{id_outputs[0], inputs[1]});
  const auto by2_outputs = cg.add(AnyFunction(by2), g_outputs);
  const FunctionGraph g = finalize(std::move(cg), by2_outputs);

  BOOST_CHECK((std::vector<graph::Term>{{0, 0}, {0, 1}}) == inputs);
  BOOST_CHECK((std::vector<graph::Term>{{1, 0}}) == id_outputs);
  BOOST_CHECK((std::vector<graph::Term>{{3, 0}}) == g_outputs);
  BOOST_CHECK((std::vector<graph::Term>{{4, 0}}) == by2_outputs);

  const std::vector<std::vector<graph::Edge>> expected_edges{
      {{0, {1, 0}}, {1, {3, 1}}}, {{0, {2, 0}}}, {{0, {3, 0}}}, {{0, {4, 0}}}, {{0, {5, 0}}}, {}};

  BOOST_CHECK(std::equal(expected_edges.begin(), expected_edges.end(), g.begin(), g.end(),
                         compare_edges_to_nodes));
}
