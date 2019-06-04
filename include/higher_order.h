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

template <size_t Base, typename... AnyTypes, typename Graph, typename Executor, typename... Inputs,
          std::size_t... Is>
auto invoke_graph(const Graph& graph, Executor& executor, AnyFunction::InvokeInput& const_inputs,
                  std::index_sequence<Is...>, Inputs... inputs) {
  return execute_graph(graph, executor, std::move(inputs)...,
                       std::any_cast<std::tuple_element_t<Is, std::tuple<AnyTypes...>>>(
                           *const_inputs[Base + Is])...);
}

template <typename Container, typename Ret, typename... ConstInputs>
struct MapFunc : graph::VirtualFunc {
  MapFunc(FunctionGraph<std::tuple<Ret>, std::tuple<typename Container::value_type, ConstInputs...>>
              graph,
          int num_inputs)
      : graph::VirtualFunc(num_inputs), graph(std::move(graph)) {}

  FunctionGraph<std::tuple<Ret>, std::tuple<typename Container::value_type, ConstInputs...>> graph;

  AnyFunction::InvokeResult operator()(graph::VirtualExecutor& executor,
                                       AnyFunction::InvokeInput&& inputs) const override {
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

    return AnyFunction::InvokeResult{std::move(results)};
  }
};

} // namespace details

template <typename Ret, typename Container, typename... ConstInputs>
auto map(
    FunctionGraph<std::tuple<Ret>, std::tuple<typename Container::value_type, ConstInputs...>> fg,
    Edge<Container> vec_edge, Edge<ConstInputs>... const_edges) {
  check_edges(vec_edge, const_edges...);

  std::shared_ptr<graph::VirtualFunc> map_node =
      std::make_shared<details::MapFunc<Container, Ret, ConstInputs...>>(
          std::move(fg), static_cast<int>(sizeof...(ConstInputs) + 1));

  std::vector<graph::Node>* nodes = vec_edge.nodes;

  add_edges(make_types<std::tuple<typename Container::value_type, ConstInputs...>>(), *nodes,
            std::tuple(vec_edge, const_edges...),
            std::make_index_sequence<sizeof...(ConstInputs) + 1>());
  nodes->emplace_back(std::move(map_node));

  return Edge<std::vector<Ret>>{nodes, {static_cast<int>(nodes->size() - 1), 0}};
}

template <typename F, typename Container, typename... ConstInputs>
auto map(F f, Edge<Container> vec_edge, Edge<ConstInputs>... const_edges) {
  using namespace traits;
  using Ret = typename function_traits<F>::return_type;
  using Args = typename function_traits<F>::args;
  using MapType = typename Container::value_type;

  static_assert(std::is_same_v<std::decay_t<std::tuple_element_t<0, Args>>, MapType>);

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
  using namespace traits;
  using Ret = typename function_traits<F>::return_type;
  using Args = typename function_traits<F>::args;
  using MapType = typename Container::value_type;

  static_assert(std::is_same_v<std::decay_t<std::tuple_element_t<0, Args>>, T>);
  static_assert(std::is_same_v<std::decay_t<std::tuple_element_t<1, Args>>, MapType>);
  static_assert(std::is_same_v<Ret, T>);

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

#endif