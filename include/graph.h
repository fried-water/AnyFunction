#pragma once

#include "any_function.h"

#include "traits.h"

#include <memory>
#include <optional>
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
    return std::tie(t1.node_id, t1.arg_idx) == std::tie(t2.node_id, t2.arg_idx);
  }
};

struct NodeEdge {
  int src_arg;
  Term dst;
  PassBy pb;

  friend bool operator==(const NodeEdge e1, const NodeEdge e2) {
    return std::tie(e1.src_arg, e1.dst, e1.pb) == std::tie(e2.src_arg, e2.dst, e2.pb);
  }
};

struct VirtualExecutor {
  virtual ~VirtualExecutor() = default;
  virtual void async(int tg, std::function<void()> func) = 0;
  virtual int create_task_group() = 0;
  virtual void wait_for_task_group(int) = 0;
};

template <typename Executor>
struct VirtualExecutorImpl : VirtualExecutor {
  Executor& executor;

  VirtualExecutorImpl(Executor& executor) : executor(executor) {}

  void async(int tg, std::function<void()> func) override { executor.async(tg, std::move(func)); }
  int create_task_group() override { return executor.create_task_group(); }
  void wait_for_task_group(int tg) override { executor.wait_for_task_group(tg); }
};

struct VirtualFunc {
  int num_inputs;
  VirtualFunc(int num_inputs) : num_inputs(num_inputs) {}
  virtual ~VirtualFunc() = default;
  virtual AnyFunction::InvokeResult operator()(VirtualExecutor& executor,
                                               AnyFunction::InvokeInput&& inputs) const = 0;
};

struct Execution {
  std::variant<AnyFunction, std::shared_ptr<VirtualFunc>> variant;

  Execution(AnyFunction any_function) : variant(std::move(any_function)) {}
  Execution(std::shared_ptr<VirtualFunc> virt_func) : variant(std::move(virt_func)) {}

  template <typename Executor>
  AnyFunction::InvokeResult operator()(Executor& executor,
                                       AnyFunction::InvokeInput&& inputs) const {
    if(std::holds_alternative<AnyFunction>(variant)) {
      return std::get<AnyFunction>(variant)(std::move(inputs));
    } else {
      VirtualExecutorImpl virtual_executor{executor};
      return (*std::get<std::shared_ptr<VirtualFunc>>(variant))(virtual_executor,
                                                                std::move(inputs));
    }
  }

  int num_inputs() const {
    return std::visit(
        [&](const auto& arg) {
          using T = std::decay_t<decltype(arg)>;
          if constexpr(std::is_same_v<T, AnyFunction>) {
            return static_cast<int>(arg.input_types().size());
          } else {
            return arg->num_inputs;
          }
        },
        variant);
  }
};

struct Node {
  std::optional<Execution> func = {};
  SmallVec<NodeEdge, 1> outputs = {};
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
  explicit FunctionGraph(ConstructingGraph<std::tuple<Inputs...>> cg, Edge<Outputs>... edges);

  const std::vector<graph::Node>& nodes() const { return _nodes; }

  std::conditional_t<sizeof...(Outputs) == 1, Edge<traits::first_t<Outputs...>>,
                     std::tuple<Edge<Outputs>...>>
  operator()(Edge<Inputs>... edges);

private:
  std::vector<graph::Node> _nodes;
};

template <typename... Outputs, typename... Inputs>
FunctionGraph(ConstructingGraph<std::tuple<Inputs...>> cg, Edge<Outputs>... edges)
    -> FunctionGraph<std::tuple<Outputs...>, std::tuple<Inputs...>>;

constexpr PassBy pass_by_of(const Type& type) {
  if(type.is_ref()) {
    return PassBy::ref;
  } else if(!type.is_copy_constructible()) {
    return PassBy::move;
  }
  return PassBy::copy;
}

// Ensure all edges came from same graph
template <typename... Ts>
void check_edges(Edge<Ts>... edges) {
  std::array<std::vector<graph::Node>*, sizeof...(Ts)> ptrs{edges.nodes...};
  assert(std::adjacent_find(ptrs.begin(), ptrs.end(), std::not_equal_to<>()) == ptrs.end());
}

template <typename... Outputs, std::size_t... Is>
std::tuple<Edge<Outputs>...> output_edges(std::vector<graph::Node>* nodes,
                                          std::index_sequence<Is...>) {
  return std::make_tuple(Edge<Outputs>{nodes, {static_cast<int>(nodes->size() - 1), Is}}...);
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
  std::unique_ptr<std::vector<graph::Node>> _nodes = std::make_unique<std::vector<graph::Node>>();

  template <typename... Ts>
  friend std::tuple<ConstructingGraph<std::tuple<Ts...>>, Edge<Ts>...> make_graph();

  template <typename FOutputs, typename FInputs>
  friend class FunctionGraph;

public:
  const std::vector<graph::Node>& nodes() const { return *_nodes; }
};

template <typename Outputs, typename Inputs>
class Delayed;

template <typename... Outputs, typename... Inputs>
class Delayed<std::tuple<Outputs...>, std::tuple<Inputs...>> {
  static_assert(sizeof...(Inputs) > 0);

  AnyFunction _any_function;

  static auto add_to_graph(AnyFunction any_function, Edge<Inputs>... edges) {
    check_edges(edges...); // all edges must come from the same graph
    std::vector<graph::Node>* nodes = std::get<0>(std::tie(edges...)).nodes;
    add_edges(any_function.input_types(), *nodes, std::tuple(std::move(edges)...),
              std::make_index_sequence<sizeof...(Inputs)>());
    nodes->push_back(graph::Node{std::move(any_function)});

    if constexpr(sizeof...(Outputs) == 1) {
      return Edge<std::tuple_element_t<0, std::tuple<Outputs...>>>{
          nodes, {static_cast<int>(nodes->size() - 1), 0}};
    } else {
      return output_edges<Outputs...>(nodes, std::make_index_sequence<sizeof...(Outputs)>());
    }
  }

public:
  template <typename F>
  Delayed(F f) : _any_function(std::move(f)) {}

  auto operator()(Edge<Inputs>... inputs) && {
    return add_to_graph(std::move(_any_function), inputs...);
  }

  auto operator()(Edge<Inputs>... inputs) const& { return add_to_graph(_any_function, inputs...); }
};

template <typename F>
Delayed(F f) -> Delayed<traits::tuple_wrap_t<traits::function_return_t<F>>,
                        traits::tuple_map_t<std::decay, traits::function_args_t<F>>>;

template <typename... Inputs>
std::tuple<ConstructingGraph<std::tuple<Inputs...>>, Edge<Inputs>...> make_graph() {
  static_assert(sizeof...(Inputs) > 0);
  ConstructingGraph<std::tuple<Inputs...>> g;
  auto nodes = g._nodes.get();
  nodes->emplace_back();
  return std::tuple_cat(
      std::make_tuple(std::move(g)),
      output_edges<Inputs...>(nodes, std::make_index_sequence<sizeof...(Inputs)>()));
}

template <typename... Outputs, typename... Inputs>
FunctionGraph<std::tuple<Outputs...>, std::tuple<Inputs...>>::FunctionGraph(
    ConstructingGraph<std::tuple<Inputs...>> cg, Edge<Outputs>... edges) {
  assert(((edges.nodes == cg._nodes.get()) && ...)); // all edges must come from cg
  add_edges(std::array<Type, sizeof...(Outputs)>{make_type<Outputs>()...}, *cg._nodes,
            std::tuple(std::move(edges)...), std::make_index_sequence<sizeof...(Outputs)>());
  cg._nodes->emplace_back();
  _nodes = std::move(*cg._nodes);
}

template <typename... Outputs, typename... Inputs>
std::conditional_t<sizeof...(Outputs) == 1, Edge<traits::first_t<Outputs...>>,
                   std::tuple<Edge<Outputs>...>>
FunctionGraph<std::tuple<Outputs...>, std::tuple<Inputs...>>::operator()(Edge<Inputs>... edges) {
  using namespace graph;
  assert((edges.nodes == ...));
  std::vector<Node>* other_nodes = std::get<0>(std::tie(edges...)).nodes;

  std::array<Term, sizeof...(Inputs)> inputs{edges.term...};

  const int index_offset = other_nodes->size() - 1;

  for(int i = 1; i < static_cast<int>(nodes().size()) - 1; i++) {
    const Node& node = nodes()[i];

    assert(node.func);
    other_nodes->push_back(graph::Node{node.func});

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
    return Edge<traits::first_t<Outputs...>>{other_nodes, outputs[0]};
  } else {
    return output_edges<Outputs...>(other_nodes, outputs,
                                    std::make_index_sequence<sizeof...(Outputs)>());
  }
}

} // namespace anyf
