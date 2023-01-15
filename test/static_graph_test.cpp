#include "anyf/static_graph.h"

#include <boost/test/unit_test.hpp>

namespace anyf {
namespace {

int identity(int x) { return x; };
int cidentity(const int& x) { return x; };
int sum(int x, int y) { return x + y; };

bool eq_without_function(const FunctionGraph& x, const FunctionGraph& y) {
  return x.input_types == y.input_types && x.output_types == y.output_types && x.owned_fwds == y.owned_fwds &&
         x.input_borrowed_fwds == y.input_borrowed_fwds && x.input_counts == y.input_counts;
}

} // namespace

BOOST_AUTO_TEST_CASE(simple_graph) {
  auto [cg, in1, in2] = make_graph<int, int>();
  const auto g = finalize(std::move(cg), Delayed(sum)(Delayed(cidentity)(in1), in2));

  auto [e_cg, inputs] = make_graph(make_type_properties(TypeList<int, int>{}));
  const auto outputs =
    e_cg.add(AnyFunction(sum), {e_cg.add(AnyFunction(cidentity), {inputs[0]}).value()[0], inputs[1]});
  const auto exp = std::move(e_cg).finalize(outputs.value()).value();

  BOOST_CHECK(eq_without_function(exp, g));
}

BOOST_AUTO_TEST_CASE(inner_graph) {
  auto [inner_cg, iin1, iin2] = make_graph<int, int>();
  const auto inner_g = finalize(std::move(inner_cg), Delayed(sum)(Delayed(cidentity)(iin1), iin2));

  auto [cg, in1, in2] = make_graph<int, int>();
  const auto g = finalize(std::move(cg), inner_g(Delayed(identity)(in1), Delayed(cidentity)(in2)));

  auto [e_cg, inputs] = make_graph(make_type_properties(TypeList<int, int>{}));
  const auto outputs = e_cg.add(inner_g,
                                {e_cg.add(AnyFunction(identity), {inputs[0]}).value()[0],
                                 e_cg.add(AnyFunction(cidentity), {inputs[1]}).value()[0]});
  const auto exp = std::move(e_cg).finalize(outputs.value()).value();

  BOOST_CHECK(eq_without_function(exp, g));
}

} // namespace anyf
