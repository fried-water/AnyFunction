#ifndef HIGHER_ORDER_H
#define HIGHER_ORDER_H

#include "graph.h"
#include "traits.h"

#include <algorithm>
#include <functional>
#include <tuple>
#include <type_traits>

namespace anyf {

// Reduce
// Filter
// Select
// Converge

namespace details {

// Ensure all edges came from same graph
template <typename... Ts>
void check_edges(Edge<Ts>... edges) {
  std::array<std::vector<graph::Node>*, sizeof...(Ts)> ptrs{edges.nodes...};
  assert(std::adjacent_find(ptrs.begin(), ptrs.end(), std::not_equal_to<>()) == ptrs.end());
}

} // namespace details

template <typename F, typename Container, typename... ConstInputs>
auto map(F f, Edge<Container> vec_edge, Edge<ConstInputs>... const_edges) {
  using namespace traits;
  using Ret = typename function_traits<F>::return_type;
  using Args = typename function_traits<F>::args;
  using MapType = typename Container::value_type;

  static_assert(std::is_same_v<std::decay_t<std::tuple_element_t<0, Args>>, MapType>);

  details::check_edges(vec_edge, const_edges...);

  auto mapped_func = AnyFunction([f = std::move(f)](Container cont, const ConstInputs&... inputs) {
    std::vector<Ret> result;
    result.reserve(cont.size());
    std::transform(cont.begin(), cont.end(), std::back_inserter(result),
                   [&](MapType& value) { return f(std::move(value), inputs...); });
    return result;
  });

  std::vector<graph::Node>* nodes = vec_edge.nodes;

  add_edges(mapped_func.input_types(), *nodes, std::tuple(vec_edge, const_edges...),
            std::make_index_sequence<sizeof...(ConstInputs) + 1>());
  nodes->emplace_back(std::move(mapped_func));

  return Edge<std::vector<Ret>>{nodes, {static_cast<int>(nodes->size() - 1), 0}};
}

template <typename F, typename Container, typename T, typename... ConstInputs>
auto accumulate(F f, Edge<Container> vec_edge, Edge<T> init_edge,
                Edge<ConstInputs>... const_edges) {
  using namespace traits;
  using Ret = typename function_traits<F>::return_type;
  using Args = typename function_traits<F>::args;
  using MapType = typename Container::value_type;

  static_assert(std::is_same_v<std::decay_t<std::tuple_element_t<0, Args>>, T>);
  static_assert(std::is_same_v<std::decay_t<std::tuple_element_t<1, Args>>, MapType>);
  static_assert(std::is_same_v<Ret, T>);

  details::check_edges(vec_edge, const_edges...);

  auto accumulate_func =
      AnyFunction([f = std::move(f)](Container cont, T init, const ConstInputs&... inputs) {
        for(auto& ele : cont) {
          init = f(std::move(init), std::move(ele), inputs...);
        }
        return init;
      });

  std::vector<graph::Node>* nodes = vec_edge.nodes;

  add_edges(accumulate_func.input_types(), *nodes, std::tuple(vec_edge, init_edge, const_edges...),
            std::make_index_sequence<sizeof...(ConstInputs) + 2>());
  nodes->emplace_back(std::move(accumulate_func));

  return Edge<T>{nodes, {static_cast<int>(nodes->size() - 1), 0}};
}

} // namespace anyf

#endif