#ifndef GRAPH_NEW_H
#define GRAPH_NEW_H

#include "any_function.h"

#include "traits.h"

#include <memory>
#include <variant>
#include <vector>

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
  Term src;
  Term dst;
  PassBy pb;

  friend bool operator==(const NodeEdge e1, const NodeEdge e2) {
    return e1.src == e2.src && e1.dst == e2.dst && e1.pb == e2.pb;
  }
};

struct Source {
  SmallVec<Type, 3> types;
  SmallVecBase<Type> const& get_types() const { return types; }
};

struct Sink {
  SmallVec<Type, 1> types;
  SmallVecBase<Type> const& get_types() const { return types; }
};

struct FuncNode {
  AnyFunction func;
  SmallVecBase<Type> const& get_types() const { return func.output_types(); }
};

struct Node {
  using VariantT = std::variant<Source, FuncNode, Sink>;

  VariantT variant;
  bool no_copy;
  SmallVec<NodeEdge, 2> outputs;

  Node(VariantT variant, bool no_copy_ = false) : variant(std::move(variant)), no_copy(no_copy_) {}

  const SmallVecBase<Type>& types() const {
    return std::visit([](const auto& arg) -> decltype(auto) { return arg.get_types(); }, variant);
  }
};

} // namespace graph

template <typename Inputs>
class ConstructingGraph;


template <typename Outputs, typename Inputs>
class FunctionGraph {
public:
  friend class ConstructingGraph<Inputs>;
  const std::vector<graph::Node>& nodes() const { return _nodes; }

private:
  explicit FunctionGraph(std::vector<graph::Node> nodes) : _nodes(std::move(nodes)) {}
  std::vector<graph::Node> _nodes;
};


template<typename Outputs, typename Inputs>
class GraphFunction;

template <typename T>
class Edge {
  Edge(std::vector<graph::Node>* nodes, int arg_idx)
    : _nodes{nodes}, _term{static_cast<int>(nodes->size() - 1), arg_idx} {}
  std::vector<graph::Node>* _nodes;
  graph::Term _term;

  template <typename Outputs, typename Inputs>
  friend class GraphFunction;

  template <typename... Inputs, typename Types, std::size_t... Is>
  friend void add_edges(const Types& types, std::vector<graph::Node>&, std::tuple<Edge<Inputs>...>, std::index_sequence<Is...>);

  template <typename... Outputs, std::size_t... Is>
  friend std::tuple<Edge<Outputs>...> output_edges(std::vector<graph::Node>*, std::index_sequence<Is...>);
};

PassBy pass_by_of(const Type& type) {
  if(type.is_ref()) {
    return PassBy::ref;
  } else if(!type.is_copy_constructible()) {
    return PassBy::move;
  }
  return PassBy::copy;
}

template <typename... Outputs, std::size_t... Is>
std::tuple<Edge<Outputs>...> output_edges(std::vector<graph::Node>* nodes, std::index_sequence<Is...>) {
  return std::tuple(Edge<Outputs>{nodes, Is}...);
}

template <typename... Inputs, typename Types, std::size_t... Is>
void add_edges(const Types& types, std::vector<graph::Node>& nodes, std::tuple<Edge<Inputs>...> edges, std::index_sequence<Is...>) {
  (nodes[std::get<Is>(edges)._term.node_id].outputs.push_back(
    graph::NodeEdge{std::get<Is>(edges)._term,
      graph::Term{static_cast<int>(nodes.size()), Is}, pass_by_of(types[Is])}), ...);
}

template <typename Inputs>
class ConstructingGraph {
  ConstructingGraph() : _nodes(std::make_unique<std::vector<graph::Node>>()) {}
  std::unique_ptr<std::vector<graph::Node>> _nodes;

  template <typename... Ts>
  friend std::tuple<ConstructingGraph<std::tuple<Ts...>>, Edge<Ts>...> make_graph();
public:
  template <typename... Outputs>
  FunctionGraph<std::tuple<Outputs...>, Inputs> outputs(Edge<Outputs>... edges) && {
    add_edges(std::array<Type, sizeof...(Outputs)>{make_type<Outputs>()...},
      *_nodes, std::tuple(std::move(edges)...), std::make_index_sequence<sizeof...(Outputs)>());
    _nodes->emplace_back(graph::Sink{{make_type<Outputs>()...}});
    return FunctionGraph<std::tuple<Outputs...>, Inputs>{std::move(*_nodes)};
  }
};

template<typename... Outputs, typename... Inputs>
class GraphFunction<std::tuple<Outputs...>, std::tuple<Inputs...>> {
  static_assert(sizeof...(Inputs) > 0);

  AnyFunction _any_function;

  static auto add_to_graph(AnyFunction any_function, Edge<Inputs>... inputs) {
    assert((inputs._nodes == ...)); // all edges must come from the same graph
    std::vector<graph::Node>* nodes = std::get<0>(std::tie(inputs...))._nodes;
    add_edges(any_function.input_types(), *nodes, std::tuple(std::move(inputs)...), std::make_index_sequence<sizeof...(Inputs)>());
    nodes->emplace_back(graph::FuncNode{std::move(any_function)}/*, TODO no_copy */);

    if constexpr(sizeof...(Outputs) == 1) {
      return Edge<std::tuple_element_t<0, std::tuple<Outputs...>>>(nodes, 0);
    } else {
      return output_edges<Outputs...>(nodes, std::make_index_sequence<sizeof...(Outputs)>());
    }
  }
public:
  template<typename F>
  GraphFunction(F f) : _any_function(make_any_function(std::move(f))) {}

  auto operator()(Edge<Inputs>... inputs) && {
    return add_to_graph(std::move(_any_function), inputs...);
  }

  auto operator()(Edge<Inputs>... inputs) const& {
    return add_to_graph(_any_function, inputs...);
  }
};

template<typename F>
auto fg(F f) {
  return GraphFunction<traits::tuple_wrap_t<traits::function_return_t<F>>, 
    traits::tuple_map_t<std::decay, traits::function_args_t<F>>>{std::move(f)};
}

template <typename... Inputs>
std::tuple<ConstructingGraph<std::tuple<Inputs...>>, Edge<Inputs>...> make_graph() {
  ConstructingGraph<std::tuple<Inputs...>> g;
  auto nodes = g._nodes.get();
  nodes->emplace_back(graph::Source{{make_type<Inputs>()...}});
  return std::tuple_cat(std::tuple(std::move(g)), output_edges<Inputs...>(nodes, std::make_index_sequence<sizeof...(Inputs)>()));
}


template <typename Outputs, typename Inputs>
struct traits::function_traits<FunctionGraph<Outputs, Inputs>> {
  using return_type = Outputs;
  using args = Inputs;

  static constexpr std::size_t arity = std::tuple_size_v<args>;
  static constexpr std::size_t num_outputs = std::tuple_size_v<tuple_wrap_t<return_type>>;
  static constexpr bool is_const = true;
};

} // namespace anyf

#endif