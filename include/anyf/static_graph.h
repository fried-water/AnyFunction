#pragma once

#include "anyf/graph.h"
#include "anyf/traits.h"

#include <memory>

namespace anyf {

template <typename T>
struct DelayedEdge {
  ConstructingGraph* cg;
  Term term;
};

template <typename... Outputs, typename T, typename... Inputs>
auto add_to_graph(T&& func, DelayedEdge<Inputs>... edges) {
  check((edges.cg == ...), "Edges don't all come from the same graph");
  ConstructingGraph* cg = std::get<0>(std::tie(edges...)).cg;

  return apply_range<sizeof...(Outputs)>(cg->add(std::forward<T>(func), std::array{edges.term...}).value(),
                                         [&](auto... terms) {
                                           return tuple_or_value(DelayedEdge<Outputs>{cg, terms}...);
                                         });
}

template <typename Outputs, typename Inputs>
class StaticFunctionGraph;

template <typename... Outputs, typename... Inputs>
struct StaticFunctionGraph<TypeList<Outputs...>, TypeList<Inputs...>> : FunctionGraph {
  auto operator()(DelayedEdge<Inputs>... edges) const { return add_to_graph<Outputs...>(*this, edges...); }
};

template <typename Inputs>
class StaticConstructingGraph;

template <typename... Inputs>
struct StaticConstructingGraph<TypeList<Inputs...>> {
  std::unique_ptr<ConstructingGraph> cg =
    std::make_unique<ConstructingGraph>(make_type_properties(TypeList<Inputs...>{}));
};

template <typename Outputs, typename Inputs>
class Delayed;

template <typename... Outputs, typename... Inputs>
class Delayed<TypeList<Outputs...>, TypeList<Inputs...>> {
  static_assert(sizeof...(Inputs) > 0);

  AnyFunction _any_function;

public:
  template <typename F>
  Delayed(F f) : _any_function(std::move(f)) {}

  auto operator()(DelayedEdge<Inputs>... inputs) && {
    return add_to_graph<Outputs...>(std::move(_any_function), inputs...);
  }

  auto operator()(DelayedEdge<Inputs>... inputs) const& { return add_to_graph<Outputs...>(_any_function, inputs...); }
};

template <typename F>
Delayed(F f) -> Delayed<decltype(decay(return_types(Type<F>{}))), decltype(decay(args(Type<F>{})))>;

template <typename... Outputs, size_t... Is>
std::tuple<DelayedEdge<Outputs>...> output_edges(ConstructingGraph* cg, std::index_sequence<Is...>) {
  return {{cg, {0, static_cast<int>(Is)}}...};
}

template <typename... Inputs>
std::tuple<StaticConstructingGraph<TypeList<Inputs...>>, DelayedEdge<Inputs>...> make_graph() {
  static_assert(sizeof...(Inputs) > 0);
  StaticConstructingGraph<TypeList<Inputs...>> g;

  ConstructingGraph* cg = g.cg.get();

  return std::tuple_cat(std::tuple(std::move(g)),
                        output_edges<Inputs...>(cg, std::make_index_sequence<sizeof...(Inputs)>()));
}

template <typename... Outputs, typename... Inputs>
StaticFunctionGraph<TypeList<Outputs...>, TypeList<Inputs...>> finalize(StaticConstructingGraph<TypeList<Inputs...>> cg,
                                                                        DelayedEdge<Outputs>... edges) {
  check(((edges.cg == cg.cg.get()) && ...), "All edges must come from the same graph");
  return {std::move(*cg.cg).finalize(std::array{edges.term...}).value()};
}

} // namespace anyf
