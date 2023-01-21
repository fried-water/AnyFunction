#include "graph_inner.h"

#include <boost/test/unit_test.hpp>

#include <algorithm>

namespace anyf {

namespace {

struct MoveOnly {
  MoveOnly(const MoveOnly&) = delete;
  MoveOnly(MoveOnly&&) = default;
};

const AnyFunction identity = AnyFunction{[](int x) { return x; }};

bool eq_without_function(const FunctionGraph::State& exp, const FunctionGraph& act) {
  const auto tie = [](const FunctionGraph::State& g) {
    return std::tie(g.input_types, g.output_types, g.owned_fwds, g.input_borrowed_fwds, g.input_counts);
  };

  return tie(exp) == tie(*act.state);
}

} // namespace

BOOST_AUTO_TEST_CASE(empty_graph) {
  auto [cg, inputs] = make_graph(make_type_properties(TypeList<>{}));
  BOOST_CHECK(eq_without_function(FunctionGraph::State{{}, {}, {{}}}, std::move(cg).finalize({}).value()));
}

BOOST_AUTO_TEST_CASE(empty_graph_value) {
  auto [cg, inputs] = make_graph(make_type_properties(TypeList<int>{}));
  const FunctionGraph g = std::move(cg).finalize({inputs[0]}).value();

  const FunctionGraph::State exp{{{type_id<int>(), true}}, {type_id<int>()}, {{{{{{0, 0, true}}, 0, 1}}}}};

  BOOST_CHECK(eq_without_function(exp, g));
}

BOOST_AUTO_TEST_CASE(empty_graph_value_fwd) {
  auto [cg, inputs] = make_graph(make_type_properties(TypeList<int>{}));
  const FunctionGraph g = std::move(cg).finalize({inputs[0]}).value();

  const FunctionGraph::State exp{{{type_id<int>(), true}}, {type_id<int>()}, {{{{{{0, 0, true}}, 0, 1}}}}};

  BOOST_CHECK(eq_without_function(exp, g));
}

BOOST_AUTO_TEST_CASE(empty_graph_value_multi_fwd) {
  auto [cg, inputs] = make_graph(make_type_properties(TypeList<int>{}));
  const FunctionGraph g = std::move(cg).finalize({inputs[0], inputs[0]}).value();

  const FunctionGraph::State exp{
    {{type_id<int>(), true}}, {type_id<int>(), type_id<int>()}, {{{{{{0, 0, true}, {0, 1, true}}, 1, 2}}}}};

  BOOST_CHECK(eq_without_function(exp, g));
}

BOOST_AUTO_TEST_CASE(empty_graph_multi_value) {
  auto [cg, inputs] = make_graph(make_type_properties(TypeList<int, int>{}));
  const FunctionGraph g = std::move(cg).finalize({inputs[0], inputs[1]}).value();

  const FunctionGraph::State exp{{{type_id<int>(), true}, {type_id<int>(), true}},
                                 {type_id<int>(), type_id<int>()},
                                 {{{{{{0, 0, true}}, 0, 1}, {{{0, 1, true}}, 0, 1}}}}};

  BOOST_CHECK(eq_without_function(exp, g));
}

BOOST_AUTO_TEST_CASE(empty_graph_value_ref) {
  auto [cg, inputs] = make_graph(make_type_properties(TypeList<int, const int&>{}));
  const FunctionGraph g = std::move(cg).finalize({inputs[0], inputs[1]}).value();

  const FunctionGraph::State exp{{{type_id<int>(), true}, {type_id<int>(), false}},
                                 {type_id<int>(), type_id<int>()},
                                 {{{{{{0, 0, true}}, 0, 1}}}},
                                 {{{{{0, 1, true}}, 1, 1}}}};

  BOOST_CHECK(eq_without_function(exp, g));
}

BOOST_AUTO_TEST_CASE(empty_graph_drop_value) {
  auto [cg, inputs] = make_graph(make_type_properties(TypeList<int>{}));
  const FunctionGraph g = std::move(cg).finalize({}).value();

  const FunctionGraph::State exp{{{type_id<int>(), true}}, {}, {{{}}}};

  BOOST_CHECK(eq_without_function(exp, g));
}

BOOST_AUTO_TEST_CASE(empty_graph_drop_ref) {
  auto [cg, inputs] = make_graph(make_type_properties(TypeList<const int&>{}));
  const FunctionGraph g = std::move(cg).finalize({}).value();

  const FunctionGraph::State exp{{{type_id<int>(), false}}, {}, {{}}, {{}}};

  BOOST_CHECK(eq_without_function(exp, g));
}

BOOST_AUTO_TEST_CASE(empty_graph_move_only_fwd) {
  auto [cg, inputs] = make_graph(make_type_properties(TypeList<MoveOnly>{}));
  const FunctionGraph g = std::move(cg).finalize({inputs[0]}).value();

  const FunctionGraph::State exp{{{type_id<MoveOnly>(), true}}, {type_id<MoveOnly>()}, {{{{{{0, 0, true}}, 0, 1}}}}};

  BOOST_CHECK(eq_without_function(exp, g));
}

BOOST_AUTO_TEST_CASE(single_value_fwd) {
  auto [cg, inputs] = make_graph(make_type_properties(TypeList<int>{}));
  const auto id_outputs = *cg.add(identity, std::array{inputs[0]});
  const FunctionGraph g = std::move(cg).finalize({id_outputs[0]}).value();

  const FunctionGraph::State exp{
    {{type_id<int>(), true}},
    {type_id<int>()},
    {{{{{{0, 0, true}}, 0, 1}}}, {{{{{1, 0, true}}, 0, 1}}}},
    {},
    {{1, 0}},
  };

  BOOST_CHECK(eq_without_function(exp, g));
}

BOOST_AUTO_TEST_CASE(single_ref_fwd) {
  auto [cg, inputs] = make_graph(make_type_properties(TypeList<const int&>{}));
  const auto id_outputs = *cg.add(AnyFunction([](const int& x) { return x; }), std::array{inputs[0]});
  const FunctionGraph g = std::move(cg).finalize({id_outputs[0]}).value();

  const FunctionGraph::State exp{{{type_id<int>(), false}},
                                 {type_id<int>()},
                                 {{}, {{{{{1, 0, true}}, 0, 1}}}},
                                 {{{{{0, 0, false}}, 0, 0}}},
                                 {{0, 1}}};

  BOOST_CHECK(eq_without_function(exp, g));
}

BOOST_AUTO_TEST_CASE(single_value_alternate_fwd) {
  auto [cg, inputs] = make_graph(make_type_properties(TypeList<int>{}));
  const auto id_outputs = *cg.add(AnyFunction([](int, const int&, int, const int&) { return 0; }),
                                  std::array{inputs[0], inputs[0], inputs[0], inputs[0]});
  const FunctionGraph g = std::move(cg).finalize({id_outputs[0]}).value();

  const FunctionGraph::State exp{
    {{type_id<int>(), true}},
    {type_id<int>()},
    {{{{{{0, 0, true}, {0, 1, true}, {0, 0, false}, {0, 1, false}}, 2, 2}}}, {{{{{1, 0, true}}, 0, 1}}}},
    {},
    {{2, 2}}};

  BOOST_CHECK(eq_without_function(exp, g));
}

BOOST_AUTO_TEST_CASE(single_ref_alternate_fwd) {
  auto [cg, inputs] = make_graph(make_type_properties(TypeList<const int&>{}));

  const auto id_outputs = *cg.add(AnyFunction([](int, const int&, int, const int&) { return 0; }),
                                  std::array{inputs[0], inputs[0], inputs[0], inputs[0]});
  const FunctionGraph g = std::move(cg).finalize({id_outputs[0]}).value();

  const FunctionGraph::State exp{{{type_id<int>(), false}},
                                 {type_id<int>()},
                                 {{}, {{{{{1, 0, true}}, 0, 1}}}},
                                 {{{{{0, 0, true}, {0, 1, true}, {0, 0, false}, {0, 1, false}}, 2, 2}}},
                                 {{2, 2}}};

  BOOST_CHECK(eq_without_function(exp, g));
}

// TODO detect circular ref/moves
BOOST_AUTO_TEST_CASE(single_move_only_fwd_ref) {
  auto [cg, inputs] = make_graph(make_type_properties(TypeList<MoveOnly>{}));
  const auto id_outputs =
    *cg.add(AnyFunction([](MoveOnly x, const MoveOnly&) { return x; }), std::array{inputs[0], inputs[0]});
  const FunctionGraph g = std::move(cg).finalize({id_outputs[0]}).value();

  const FunctionGraph::State exp{{{type_id<MoveOnly>(), true}},
                                 {type_id<MoveOnly>()},
                                 {{{{{{0, 0, true}, {0, 0, false}}, 0, 1}}}, {{{{{1, 0, true}}, 0, 1}}}},
                                 {},
                                 {{1, 1}}};

  BOOST_CHECK(eq_without_function(exp, g));
}

BOOST_AUTO_TEST_CASE(single_void) {
  auto [cg, inputs] = make_graph(make_type_properties(TypeList<>{}));
  const auto id_outputs = *cg.add(AnyFunction([]() {}), {});
  const FunctionGraph g = std::move(cg).finalize({}).value();

  const FunctionGraph::State exp{{}, {}, {{}, {}}, {}, {{0, 0}}};

  BOOST_CHECK(eq_without_function(exp, g));
}

BOOST_AUTO_TEST_CASE(oterm_type) {
  auto [cg, inputs] = make_graph(make_type_properties(TypeList<int, const int&, char, const char&>{}));
  const auto outputs = *cg.add(AnyFunction([](int x) { return std::tuple(x, 1.0f); }), std::array{inputs[0]});

  BOOST_CHECK(cg.type(inputs[0]) == (TypeProperties{type_id<int>(), true}));
  BOOST_CHECK(cg.type(inputs[1]) == (TypeProperties{type_id<int>(), false}));
  BOOST_CHECK(cg.type(inputs[2]) == (TypeProperties{type_id<char>(), true}));
  BOOST_CHECK(cg.type(inputs[3]) == (TypeProperties{type_id<char>(), false}));

  BOOST_CHECK(cg.type(outputs[0]) == (TypeProperties{type_id<int>(), true}));
  BOOST_CHECK(cg.type(outputs[1]) == (TypeProperties{type_id<float>(), true}));
}

BOOST_AUTO_TEST_CASE(graph_types) {
  const AnyFunction f([](int x, const float& y) { return std::tuple(x, y); });
  const auto g = make_graph(f);

  BOOST_CHECK(f.input_types() == g.input_types());
  BOOST_CHECK(f.output_types() == g.output_types());
}

BOOST_AUTO_TEST_CASE(add_empty_graph) {
  auto [inner_cg, inner_inputs] = make_graph(make_type_properties(TypeList<int>{}));
  const FunctionGraph inner_g = std::move(inner_cg).finalize(inner_inputs).value();

  auto [cg, inputs] = make_graph(make_type_properties(TypeList<int>{}));
  const FunctionGraph g = std::move(cg).finalize(*cg.add(inner_g, inputs)).value();

  BOOST_CHECK(eq_without_function(*inner_g.state, g));
}

BOOST_AUTO_TEST_CASE(add_single_graph) {
  auto [inner_cg, inner_inputs] = make_graph(make_type_properties(TypeList<int>{}));
  const FunctionGraph inner_g = std::move(inner_cg).finalize(*inner_cg.add(identity, inner_inputs)).value();

  auto [cg, inputs] = make_graph(make_type_properties(TypeList<int>{}));
  const FunctionGraph g = std::move(cg).finalize(*cg.add(inner_g, inputs)).value();

  BOOST_CHECK(eq_without_function(*inner_g.state, g));
}

BOOST_AUTO_TEST_CASE(add_duo_graph) {
  auto [inner_cg, inner_inputs] = make_graph(make_type_properties(TypeList<int>{}));
  const FunctionGraph inner_g =
    std::move(inner_cg).finalize(*inner_cg.add(identity, *inner_cg.add(identity, inner_inputs))).value();

  auto [cg, inputs] = make_graph(make_type_properties(TypeList<int>{}));
  const FunctionGraph g = std::move(cg).finalize(*cg.add(inner_g, inputs)).value();

  BOOST_CHECK(eq_without_function(*inner_g.state, g));
}

BOOST_AUTO_TEST_CASE(add_single_graph_to_single_after) {
  auto [inner_cg, inner_inputs] = make_graph(make_type_properties(TypeList<int>{}));
  const FunctionGraph inner_g = *std::move(inner_cg).finalize(*inner_cg.add(identity, inner_inputs));

  auto [cg, inputs] = make_graph(make_type_properties(TypeList<int>{}));
  const FunctionGraph g = *std::move(cg).finalize(*cg.add(inner_g, *cg.add(identity, inputs)));

  auto [exp_cg, exp_inputs] = make_graph(make_type_properties(TypeList<int>{}));
  const FunctionGraph exp = *std::move(exp_cg).finalize(*exp_cg.add(identity, *exp_cg.add(identity, exp_inputs)));

  BOOST_CHECK(eq_without_function(*exp.state, g));
}

BOOST_AUTO_TEST_CASE(bad_arity) {
  auto [cg, inputs] = make_graph(make_type_properties(TypeList<>{}));
  const auto error = cg.add(identity, {}).error();
  BOOST_CHECK((GraphError{BadArity{1, 0}} == error));
}

BOOST_AUTO_TEST_CASE(bad_type) {
  auto [cg, inputs] = make_graph(make_type_properties(TypeList<char>{}));
  const auto error = cg.add(identity, {inputs[0]}).error();
  BOOST_CHECK((GraphError{BadType{0, type_id<int>(), type_id<char>()}} == error));
}

BOOST_AUTO_TEST_CASE(move_only) {
  auto [cg, inputs] = make_graph(make_type_properties(TypeList<MoveOnly>{}));
  const auto error = cg.add(AnyFunction([](MoveOnly, MoveOnly) {}), {inputs[0], inputs[0]}).error();
  BOOST_CHECK((GraphError{AlreadyMoved{1, type_id<MoveOnly>()}} == error));
}

BOOST_AUTO_TEST_CASE(cannot_copy) {
  auto [cg, inputs] = make_graph(make_type_properties(TypeList<const MoveOnly&>{}));
  const auto error = cg.add(AnyFunction([](MoveOnly) {}), {inputs[0]}).error();
  BOOST_CHECK((GraphError{CannotCopy{0, type_id<MoveOnly>()}} == error));
}

BOOST_AUTO_TEST_CASE(err_offset) {
  auto [cg, inputs] = make_graph(make_type_properties(TypeList<char>{}));
  const auto error = cg.add(AnyFunction([](char, int) {}), {inputs[0], inputs[0]}).error();
  BOOST_CHECK((GraphError{BadType{1, type_id<int>(), type_id<char>()}} == error));
}

BOOST_AUTO_TEST_CASE(err_ref_offset) {
  auto [cg, inputs] = make_graph(make_type_properties(TypeList<char>{}));
  const auto error = cg.add(AnyFunction([](const char&, int) {}), {inputs[0], inputs[0]}).error();
  BOOST_CHECK((GraphError{BadType{1, type_id<int>(), type_id<char>()}} == error));
}

BOOST_AUTO_TEST_CASE(mismatched_branch_types) {
  auto [cg, inputs] = make_graph(make_type_properties(TypeList<int>{}));
  const auto error =
    cg.add_if(make_graph(AnyFunction{[](int x) { return x; }}), make_graph(AnyFunction{[]() {}}), inputs).error();
  BOOST_CHECK((GraphError{MismatchedBranchTypes{}} == error));
}

} // namespace anyf
