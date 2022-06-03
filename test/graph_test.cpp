#include "anyf/graph.h"

#include <boost/test/unit_test.hpp>

#include <algorithm>

namespace anyf {

namespace {

struct MoveOnly {
  MoveOnly(const MoveOnly&) = delete;
  MoveOnly(MoveOnly&&) = default;
};

int identity(int x) { return x; }
int cidentity(const int& x) { return x; }
int by2(const int& x) { return x * 2; }
auto sum(int x, int y) { return x + y; }

bool compare_edges_to_nodes(const std::vector<Edge>& expected, const Node& actual) {
  return std::equal(expected.begin(), expected.end(), actual.outputs.begin(), actual.outputs.end());
}

bool compare_node_outputs(const Node& expected, const Node& actual) { return expected.outputs == actual.outputs; }

} // namespace

BOOST_AUTO_TEST_CASE(simple_graph_construction) {
  auto [cg, inputs] = make_graph(make_types(TypeList<int, int>{}));
  const auto id_outputs = cg.add(AnyFunction(cidentity), std::array{inputs[0]}).value();
  const auto sum_outputs = cg.add(AnyFunction(sum), std::array{id_outputs[0], inputs[1]}).value();
  const FunctionGraph g = std::move(cg).finalize(sum_outputs).value();

  const std::vector<std::vector<Edge>> expected_edges{{{0, {1, 0}}, {1, {2, 1}}}, {{0, {2, 0}}}, {{0, {3, 0}}}, {}};

  BOOST_CHECK((std::vector<Term>{{0, 0}, {0, 1}}) == inputs);
  BOOST_CHECK((std::vector<Term>{{1, 0}}) == id_outputs);
  BOOST_CHECK((std::vector<Term>{{2, 0}}) == sum_outputs);

  BOOST_CHECK(std::equal(expected_edges.begin(), expected_edges.end(), g.begin(), g.end(), compare_edges_to_nodes));

  BOOST_CHECK(make_types(TypeList<int, int>{}) == std::get<std::vector<TypeProperties>>(g.front().func));
  BOOST_CHECK(make_types(TypeList<int>{}) == std::get<std::vector<TypeProperties>>(g.back().func));
}

BOOST_AUTO_TEST_CASE(simple_inner_graph) {
  auto [inner_cg, inner_inputs] = make_graph(make_types(TypeList<int, int>{}));
  const auto inner_id_outputs = inner_cg.add(AnyFunction(cidentity), std::array{inner_inputs[0]}).value();
  const auto inner_sum_outputs = inner_cg.add(AnyFunction(sum), std::array{inner_id_outputs[0], inner_inputs[1]}).value();
  const FunctionGraph inner_g = std::move(inner_cg).finalize(inner_sum_outputs).value();

  auto [cg, inputs] = make_graph(make_types(TypeList<int, int>{}));
  const auto id_outputs = cg.add(AnyFunction(identity), std::array{inputs[0]}).value();
  const auto g_outputs = cg.add(inner_g, std::array{id_outputs[0], inputs[1]}).value();
  const auto by2_outputs = cg.add(AnyFunction(by2), g_outputs).value();
  const FunctionGraph g = std::move(cg).finalize(by2_outputs).value();

  BOOST_CHECK((std::vector<Term>{{0, 0}, {0, 1}}) == inputs);
  BOOST_CHECK((std::vector<Term>{{1, 0}}) == id_outputs);
  BOOST_CHECK((std::vector<Term>{{3, 0}}) == g_outputs);
  BOOST_CHECK((std::vector<Term>{{4, 0}}) == by2_outputs);

  const std::vector<std::vector<Edge>> expected_edges{
    {{0, {1, 0}}, {1, {3, 1}}}, {{0, {2, 0}}}, {{0, {3, 0}}}, {{0, {4, 0}}}, {{0, {5, 0}}}, {}};

  BOOST_CHECK(std::equal(expected_edges.begin(), expected_edges.end(), g.begin(), g.end(), compare_edges_to_nodes));
}

BOOST_AUTO_TEST_CASE(graph_errors) {
  auto [cg, inputs] = make_graph(make_types(TypeList<int, char, MoveOnly>{}));
  BOOST_CHECK((GraphError{BadArity{1, 2}})
    == cg.add(AnyFunction(identity), std::array{inputs[0], inputs[0]}).error());

  BOOST_CHECK((GraphError{BadType{0, type_id(Type<int>{}), type_id(Type<char>{})}})
    == cg.add(AnyFunction(identity), std::array{inputs[1]}).error());

  BOOST_CHECK((GraphError{AlreadyMoved{1, type_id(Type<MoveOnly>{})}})
    == cg.add(AnyFunction([](MoveOnly, MoveOnly) { return 0; }), std::array{inputs[2], inputs[2]}).error());
}

} // namespace anyf
