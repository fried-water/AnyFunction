#ifndef GRAPH_H
#define GRAPH_H

#include "any_function.h"

#include "traits.h"

#include <memory>
#include <variant>
#include <vector>

// TODO ensure PassBy is Optimal and Add checks for double move
// TODO add names to Graphs and nodes

namespace anyf {

enum class PassBy { move, copy, ref };

namespace graph {

struct Term {
  int node_id;
  int arg_idx;
  friend bool operator==(const Term t1, const Term t2) {
    return t1.node_id == t2.node_id && t1.arg_idx == t2.arg_idx;
  }
};

struct NodeEdge {
  int src_arg;
  Term dst;
  PassBy pb;

  friend bool operator==(const NodeEdge e1, const NodeEdge e2) {
    return e1.src_arg == e2.src_arg && e1.dst == e2.dst && e1.pb == e2.pb;
  }
};

struct Node {
  std::optional<AnyFunction> func;
  SmallVec<NodeEdge, 2> outputs;

  Node() = default;
  Node(AnyFunction any_func) : func(std::move(any_func)) {}
};

} // namespace graph

template <typename T>
struct Edge {
  std::vector<graph::Node>* nodes;
  graph::Term term;
};

template <typename Inputs>
class ConstructingGraph;

template <typename Outputs, typename Inputs>
class FunctionGraph;

template <typename... Outputs, typename... Inputs>
class FunctionGraph<std::tuple<Outputs...>, std::tuple<Inputs...>> {
public:
  const std::vector<graph::Node>& nodes() const { return _nodes; }

  std::conditional_t<sizeof...(Outputs) == 1, Edge<Outputs...>, std::tuple<Edge<Outputs>...>>
  operator()(Edge<Inputs>... edges);

private:
  explicit FunctionGraph(std::vector<graph::Node> nodes) : _nodes(std::move(nodes)) {}
  std::vector<graph::Node> _nodes;

  friend class ConstructingGraph<std::tuple<Inputs...>>;
};

constexpr PassBy pass_by_of(const Type& type) {
  if(type.is_ref()) {
    return PassBy::ref;
  } else if(!type.is_copy_constructible()) {
    return PassBy::move;
  }
  return PassBy::copy;
}

template <typename... Outputs, std::size_t... Is>
std::tuple<Edge<Outputs>...> output_edges(std::vector<graph::Node>* nodes,
                                          std::index_sequence<Is...>) {
  return std::tuple(Edge<Outputs>{nodes, {static_cast<int>(nodes->size() - 1), Is}}...);
}

template <typename... Outputs, std::size_t... Is>
std::tuple<Edge<Outputs>...> output_edges(std::vector<graph::Node>* nodes,
                                          const std::array<graph::Term, sizeof...(Outputs)>& terms,
                                          std::index_sequence<Is...>) {
  return std::tuple(Edge<Outputs>{nodes, terms[Is]}...);
}

template <typename... Inputs, typename Types, std::size_t... Is>
void add_edges(const Types& types, std::vector<graph::Node>& nodes,
               std::tuple<Edge<Inputs>...> edges, std::index_sequence<Is...>) {
  (nodes[std::get<Is>(edges).term.node_id].outputs.push_back(
       graph::NodeEdge{std::get<Is>(edges).term.arg_idx,
                       graph::Term{static_cast<int>(nodes.size()), Is}, pass_by_of(types[Is])}),
   ...);
}

template <typename Inputs>
class ConstructingGraph {
  ConstructingGraph() : _nodes(std::make_unique<std::vector<graph::Node>>()) {}
  std::unique_ptr<std::vector<graph::Node>> _nodes;

  template <typename... Ts>
  friend std::tuple<ConstructingGraph<std::tuple<Ts...>>, Edge<Ts>...> make_graph();

public:
  const std::vector<graph::Node>& nodes() const { return *_nodes; }

  template <typename... Outputs>
  FunctionGraph<std::tuple<Outputs...>, Inputs> outputs(Edge<Outputs>... edges) && {
    assert(((edges.nodes == _nodes.get()) && ...)); // all edges must come from this graph
    add_edges(std::array<Type, sizeof...(Outputs)>{make_type<Outputs>()...}, *_nodes,
              std::tuple(std::move(edges)...), std::make_index_sequence<sizeof...(Outputs)>());
    _nodes->emplace_back();
    return FunctionGraph<std::tuple<Outputs...>, Inputs>{std::move(*_nodes)};
  }
};

template <typename Outputs, typename Inputs>
class Delayed;

template <typename... Outputs, typename... Inputs>
class Delayed<std::tuple<Outputs...>, std::tuple<Inputs...>> {
  static_assert(sizeof...(Inputs) > 0);

  AnyFunction _any_function;

  static auto add_to_graph(AnyFunction any_function, Edge<Inputs>... edges) {
    assert((edges.nodes == ...)); // all edges must come from the same graph
    std::vector<graph::Node>* nodes = std::get<0>(std::tie(edges...)).nodes;
    add_edges(any_function.input_types(), *nodes, std::tuple(std::move(edges)...),
              std::make_index_sequence<sizeof...(Inputs)>());
    nodes->emplace_back(std::move(any_function) /*, TODO no_copy */);

    if constexpr(sizeof...(Outputs) == 1) {
      return Edge<std::tuple_element_t<0, std::tuple<Outputs...>>>{
          nodes, {static_cast<int>(nodes->size() - 1), 0}};
    } else {
      return output_edges<Outputs...>(nodes, std::make_index_sequence<sizeof...(Outputs)>());
    }
  }

public:
  template <typename F>
  Delayed(F f) : _any_function(make_any_function(std::move(f))) {}

  auto operator()(Edge<Inputs>... inputs) && {
    return add_to_graph(std::move(_any_function), inputs...);
  }

  auto operator()(Edge<Inputs>... inputs) const& { return add_to_graph(_any_function, inputs...); }
};

template <typename F>
auto fg(F f) {
  return Delayed<traits::tuple_wrap_t<traits::function_return_t<F>>,
                 traits::tuple_map_t<std::decay, traits::function_args_t<F>>>{std::move(f)};
}

template <typename... Inputs>
std::tuple<ConstructingGraph<std::tuple<Inputs...>>, Edge<Inputs>...> make_graph() {
  static_assert(sizeof...(Inputs) > 0);
  ConstructingGraph<std::tuple<Inputs...>> g;
  auto nodes = g._nodes.get();
  nodes->emplace_back();
  return std::tuple_cat(
      std::tuple(std::move(g)),
      output_edges<Inputs...>(nodes, std::make_index_sequence<sizeof...(Inputs)>()));
}

template <typename... Outputs, typename... Inputs>
std::conditional_t<sizeof...(Outputs) == 1, Edge<Outputs...>, std::tuple<Edge<Outputs>...>>
FunctionGraph<std::tuple<Outputs...>, std::tuple<Inputs...>>::operator()(Edge<Inputs>... edges) {
  using namespace graph;
  assert((edges.nodes == ...));
  std::vector<Node>* other_nodes = std::get<0>(std::tie(edges...)).nodes;

  std::array<Term, sizeof...(Inputs)> inputs{edges.term...};

  const int index_offset = other_nodes->size() - 1;

  for(int i = 1; i < static_cast<int>(nodes().size()) - 1; i++) {
    const Node& node = nodes()[i];

    assert(node.func);
    other_nodes->emplace_back(*node.func);

    for(int j = 0; j < i; j++) {
      for(const NodeEdge& edge : nodes()[j].outputs) {
        if(edge.dst.node_id == i) {
          Term src = j == 0 ? inputs[edge.src_arg] : Term{j + index_offset, edge.src_arg};

          Term dst{i + index_offset, edge.dst.arg_idx};

          (*other_nodes)[src.node_id].outputs.push_back(NodeEdge{src.arg_idx, dst, edge.pb});
        }
      }
    }
  }

  std::array<Term, sizeof...(Outputs)> outputs{};

  for(int i = 0; i < static_cast<int>(nodes().size()); i++) {
    for(const NodeEdge& edge : nodes()[i].outputs) {
      if(edge.dst.node_id == static_cast<int>(nodes().size() - 1)) {
        outputs[edge.src_arg] = Term{i + index_offset, edge.src_arg};
      }
    }
  }

  if constexpr(sizeof...(Outputs) == 1) {
    return Edge<std::tuple_element_t<0, std::tuple<Outputs...>>>{other_nodes, outputs[0]};
  } else {
    return output_edges<Outputs...>(other_nodes, outputs,
                                    std::make_index_sequence<sizeof...(Outputs)>());
  }
}

} // namespace anyf

#endif