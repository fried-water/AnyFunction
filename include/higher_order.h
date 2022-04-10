#pragma once

#include "static_graph.h"
#include "traits.h"

#include <algorithm>
#include <functional>
#include <tuple>
#include <vector>

namespace anyf {

// Reduce
// Filter
// Select
// Converge

namespace details {

template <size_t Base, typename... AnyTypes, typename Graph, typename Executor, typename... Inputs,
          std::size_t... Is>
auto invoke_graph(const Graph& graph, Executor& executor, std::vector<std::any*>& const_inputs,
                  std::index_sequence<Is...>, Inputs... inputs) {
  return execute_graph(graph, executor, std::move(inputs)...,
                       std::any_cast<std::tuple_element_t<Is, std::tuple<AnyTypes...>>>(
                           *const_inputs[Base + Is])...);
}

template <typename Container, typename Ret, typename... ConstInputs>
struct MapFunc : graph::VirtualFunc {
  MapFunc(FunctionGraph<TL<Ret>, TL<typename Container::value_type, ConstInputs...>> graph,
          int num_inputs)
      : graph::VirtualFunc(num_inputs), graph(std::move(graph)) {}

  FunctionGraph<TL<Ret>, TL<typename Container::value_type, ConstInputs...>> graph;

  std::vector<std::any> operator()(graph::VirtualExecutor& executor,
                                   std::vector<std::any*>&& inputs) const override {
    int tg_id = executor.create_task_group();

    Container container = std::any_cast<Container>(std::move(*inputs[0]));
    std::vector<Ret> results(container.size());

    int i = 0;
    for(auto& input : container) {
      executor.async(tg_id, [&, i]() {
        results[i] = details::invoke_graph<1, ConstInputs...>(
            graph, executor, inputs, std::make_index_sequence<sizeof...(ConstInputs)>{},
            std::move(input));
      });
      i++;
    }

    executor.wait_for_task_group(tg_id);

    return make_vector<std::any>(std::move(results));
  }
};

} // namespace details

template <typename Ret, typename Container, typename... ConstInputs>
auto map(FunctionGraph<TL<Ret>, TL<typename Container::value_type, ConstInputs...>> fg,
         Edge<Container> vec_edge, Edge<ConstInputs>... const_edges) {
  check_edges(vec_edge, const_edges...);

  std::shared_ptr<graph::VirtualFunc> map_node =
      std::make_shared<details::MapFunc<Container, Ret, ConstInputs...>>(
          std::move(fg), static_cast<int>(sizeof...(ConstInputs) + 1));

  std::vector<graph::Node>* nodes = vec_edge.nodes;

  add_edges(make_types(TL<typename Container::value_type, ConstInputs...>{}), *nodes,
            std::tuple(vec_edge, const_edges...),
            std::make_index_sequence<sizeof...(ConstInputs) + 1>());
  nodes->push_back(graph::Node{std::move(map_node)});

  return Edge<std::vector<Ret>>{nodes, {static_cast<int>(nodes->size() - 1), 0}};
}

template <typename F, typename Container, typename... ConstInputs>
auto map(F f, Edge<Container> vec_edge, Edge<ConstInputs>... const_edges) {
  using Ret = typename decltype(return_type<F>())::type;
  using MapType = typename Container::value_type;

  static_assert(is_same(decay(head(args<F>())), Ty<MapType>{}));

  check_edges(vec_edge, const_edges...);

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
  static_assert(is_same(decay(head(args<F>())), Ty<T>{}));
  static_assert(is_same(decay(head(tail(args<F>()))), Ty<typename Container::value_type>{}));
  static_assert(is_same(return_type<F>(), Ty<T>{}));

  check_edges(vec_edge, const_edges...);

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
