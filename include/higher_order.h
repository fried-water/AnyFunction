#ifndef HIGHER_ORDER_H
#define HIGHER_ORDER_H

#include "graph.h"
#include "traits.h"

#include <functional>
#include <tuple>
#include <type_traits>

#include <iostream>

namespace anyf {

// Map
// For each
// Reduce
// Select
// While

template <typename F, typename Container, typename... ConstInputs>
auto map(F f, Edge<Container> vec_edge, Edge<ConstInputs>... const_edges) {
  using namespace traits;
  using Ret = typename function_traits<F>::return_type;
  using Args = typename function_traits<F>::args;
  using MapType = typename Container::value_type;

  static_assert(std::is_same_v<std::decay_t<std::tuple_element_t<0, Args>>, MapType>);

  auto mapped_func = AnyFunction([f = std::move(f)](Container cont, const ConstInputs&... inputs) {
    std::vector<Ret> result;
    result.reserve(cont.size());
    std::transform(cont.begin(), cont.end(), std::back_inserter(result),
                   [&](MapType& value) { return f(std::move(value), inputs...); });
    return result;
  });

  // assert((const_edges.nodes == ...)); // all edges must come from the same graph
  // TODO check container edge

  std::vector<graph::Node>* nodes = vec_edge.nodes;

  add_edges(mapped_func.input_types(), *nodes, std::tuple(vec_edge, const_edges...),
            std::make_index_sequence<sizeof...(ConstInputs) + 1>());
  nodes->emplace_back(std::move(mapped_func));

  return Edge<std::vector<Ret>>{nodes, {static_cast<int>(nodes->size() - 1), 0}};
}

} // namespace anyf

#endif