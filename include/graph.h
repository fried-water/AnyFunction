#ifndef GRAPH_H
#define GRAPH_H

#include "any_function.h"
#include "util.h"

#include <cctype>

#include <iostream>
#include <numeric>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <variant>

#include <boost/container/small_vector.hpp>
#include <boost/range/adaptor/indexed.hpp>
#include <boost/range/algorithm/find_if.hpp>
#include <boost/range/algorithm/for_each.hpp>
#include <boost/range/algorithm/transform.hpp>
#include <boost/range/combine.hpp>

namespace anyf {

struct BadConstruction : public std::exception {
  std::string msg;
  BadConstruction(std::string msg_) : msg{msg_} {}
  const char* what() const noexcept override { return msg.c_str(); }
};

enum class PassBy { move, copy, ref };

namespace graph {

struct Term {
  int node_id;
  int arg_idx;
  friend bool operator==(const Term t1, const Term t2) {
    return t1.node_id == t2.node_id && t1.arg_idx == t2.arg_idx;
  }
};

struct Edge {
  Term src;
  Term dst;
  PassBy pb;

  friend bool operator==(const Edge e1, const Edge e2) {
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

inline bool is_valid_name(const std::string_view name) {
  return name.size() != 0 &&
         std::isalpha(name.front()) &&
         std::all_of(name.begin(), name.end(), [](char c) { return std::isalnum(c) || c == '_'; });
}

struct Node {
  using VariantT = std::variant<Source, FuncNode, Sink>;

  VariantT variant;
  bool no_copy = false;
  std::string name;

  SmallVec<std::string, 1> output_names;
  SmallVec<Edge, 2> outputs;

  Node(Source src, std::string name, SmallVec<std::string, 1> output_names)
      : Node(VariantT(std::move(src)), std::move(name), std::move(output_names)) {}
  Node(FuncNode FuncNode, std::string name, SmallVec<std::string, 1> output_names)
      : Node(VariantT(std::move(FuncNode)), std::move(name), std::move(output_names)) {}
  Node(Sink Sink) : variant(std::move(Sink)) {}

  SmallVecBase<Type> const& types() const {
    return std::visit([](const auto& arg) -> decltype(auto) { return arg.get_types(); }, variant);
  }
private:
  Node(VariantT variant, std::string name_, SmallVec<std::string, 1> output_names)
      : variant(std::move(variant)), no_copy(name_.find("[nocopy]") == 0), name(std::move(name_)),
        output_names(std::move(output_names)) {
    if(no_copy) {
      name.erase(0, strlen("[nocopy]"));
    }
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
  FunctionGraph(std::vector<graph::Node> Nodes) : _nodes(std::move(Nodes)) {}
  std::vector<graph::Node> _nodes;
};

template <typename Inputs>
class ConstructingGraph {
public:
  template <typename F>
  ConstructingGraph& add(F f, std::string name, SmallVec<std::string, 3> inputs, SmallVec<std::string, 1> outputs = {}) {
    if(!graph::is_valid_name(name)) {
      throw BadConstruction{"Invalid function name: " + name};
    }

    // Check all output names are valid
    std::for_each(outputs.begin(), outputs.end(), [](const auto& name) {
      if(!graph::is_valid_name(name))
        throw BadConstruction{"Invalid output: " + name};
    });

    auto any_func = make_any_function(std::move(f));

    if(any_func.output_types().size() != outputs.size() &&
       !(any_func.output_types().size() == 1 && outputs.size() == 0)) {
      throw BadConstruction{"Incorrect number of output names for " + name + ": " + std::to_string(outputs.size())};
    }

    return add_internal(std::move(any_func), std::move(name),
                        util::map<SmallVec<graph::Term, 3>>(
                            std::move(inputs),
                            [this](const std::string& name) {
                              auto opt_idx = idx_of(name);
                              if(!opt_idx)
                                throw BadConstruction{"Input not found: " + std::string(name)};
                              return *opt_idx;
                            }),
                        std::move(outputs));
  }

  template <typename... InnerOutputs, typename... InnerInputs>
  ConstructingGraph& add(const FunctionGraph<std::tuple<InnerOutputs...>, std::tuple<InnerInputs...>>& graph,
                         std::string name, SmallVec<std::string, 3> inputs, SmallVec<std::string, 3> output_names = {})
  {
    using namespace graph;

    if(sizeof...(InnerOutputs) != output_names.size() && !(sizeof...(InnerOutputs) == 1 && output_names.size() == 0)) {
      throw BadConstruction{"Incorrect number of output names for " + name + ": " + std::to_string(output_names.size())};
    }

    const int index_offset = _nodes.size() - 1;
    SmallVec<Term, 3> input_terms;

    for(int i = 1; i < static_cast<int>(graph.nodes().size()) - 1; i++) {
      const Node& node = graph.nodes()[i];

      assert(std::holds_alternative<FuncNode>(node.variant));

      input_terms.clear();

      for(int j = 0; j < i; j++) {
        for(const Edge& edge : graph.nodes()[j].outputs) {
          if(edge.dst.node_id == i) {
            input_terms.push_back(edge.src);
          }
        }
      }

      add_internal(
          std::get<FuncNode>(node.variant).func, name + "." + node.name,
          util::map<SmallVec<Term, 3>>(std::move(input_terms), [&](Term old_term) {
            if(old_term.node_id == 0) {
              auto opt_idx = idx_of(inputs[old_term.arg_idx]);
              if(!opt_idx) throw BadConstruction("Invalid input: " + inputs[old_term.arg_idx]);
              return *opt_idx;
            } else {
              return Term{old_term.node_id + index_offset, old_term.arg_idx};
            }
          }), {});
    }

    SmallVec<Term, 1> output_terms;
    for(const Node& node : graph.nodes()) {
      for(const Edge& edge : node.outputs) {
        if(edge.dst.node_id == static_cast<int>(graph.nodes().size() - 1)) {
          output_terms.push_back(Term{edge.src.node_id + index_offset, edge.src.arg_idx});
        }
      }
    }

    if(output_names.size() == 0) {
      assert(output_terms.size() == 1);
      _aliases.emplace(name, output_terms.front());
    } else {
      assert(output_terms.size() == output_names.size());
      boost::transform(boost::combine(output_names, output_terms), std::inserter(_aliases, _aliases.end()), 
        [index_offset, &name](const auto& tuple) {
          const auto& term_name = boost::get<0>(tuple);
          const auto& term = boost::get<1>(tuple);
          return std::make_pair(name + "." + term_name, term);
        });
    }

    return *this;
  }

  template <typename... Outputs>
  FunctionGraph<std::tuple<Outputs...>, Inputs> output(SmallVec<std::string, 3> input_names) {
    boost::for_each(input_names | boost::adaptors::indexed(), [this](const auto& ele) {
      const auto& name = ele.value();
      auto opt_term = idx_of(name);
      if(!opt_term) throw BadConstruction{"Invalid input: " + std::string(name)};

      auto term = *opt_term;
      graph::Node& src = _nodes[term.node_id];

      if(src.no_copy) {
        auto it = boost::find_if(src.outputs, [](const auto& Edge) {
                 return Edge.pb == PassBy::move;
               });

        if(it != src.outputs.end()) {
          throw BadConstruction{"Invalid input: " + std::string(name)};
        }
      }

      src.outputs.push_back(graph::Edge{term,
        graph::Term{static_cast<int>(_nodes.size()), static_cast<int>(ele.index())},
        src.no_copy ? PassBy::move : PassBy::copy});
    });
    _nodes.emplace_back(graph::Sink{{make_type<Outputs>()...}});

    return FunctionGraph<std::tuple<Outputs...>, Inputs>(std::move(_nodes));
  }
private:
  std::vector<graph::Node> _nodes;
  std::unordered_map<std::string, graph::Term> _aliases;

  ConstructingGraph(std::array<std::string, std::tuple_size_v<Inputs>> input_names, SmallVec<Type, 3> types) {
    _nodes.reserve(5); // Arbitrary, but allows for src, sink, and 3 inner nodes

    // Check all input names are valid
    std::for_each(input_names.begin(), input_names.end(), [](const auto& name) {
      if(!graph::is_valid_name(name))
        throw BadConstruction{"Invalid input: " + name};
    });

    _nodes.emplace_back(graph::Source{std::move(types)}, "",
                        SmallVec<std::string, 1>{std::make_move_iterator(input_names.begin()),
                                                 std::make_move_iterator(input_names.end())});
  }

  std::optional<graph::Term> idx_of(const std::string& name) const {
    
    auto alias_it = _aliases.find(name);

    if(alias_it != _aliases.end()) {
      return alias_it->second;
    }

    auto split_pos = name.find('.');
    if(split_pos == std::string_view::npos) {
      auto it = std::find_if(_nodes.begin(), _nodes.end(),
                             [&name](const auto& node) { return node.name == name; });

      if(it == _nodes.end() || it->output_names.size() > 1)
        return std::nullopt;

      return graph::Term{static_cast<int>(std::distance(_nodes.begin(), it)), 0};
    }

    const std::string_view node_name = name.substr(0, split_pos);
    const std::string_view output_name = name.substr(split_pos + 1);

    if((!graph::is_valid_name(node_name) && !node_name.empty()) ||
       !graph::is_valid_name(output_name)) {
      return std::nullopt;
    }

    auto it = std::find_if(_nodes.begin(), _nodes.end(),
                           [&node_name](const auto& node) { return node.name == node_name; });

    if(it == _nodes.end())
      return std::nullopt;

    auto output_it = std::find(it->output_names.begin(), it->output_names.end(), output_name);

    if(output_it == it->output_names.end())
      return std::nullopt;

    return graph::Term{static_cast<int>(std::distance(_nodes.begin(), it)),
                       static_cast<int>(std::distance(it->output_names.begin(), output_it))};
  }

  ConstructingGraph& add_internal(AnyFunction f, std::string name, SmallVec<graph::Term, 3> inputs,
                                  SmallVec<std::string, 1> output_names)
  {
    auto it = std::find_if(_nodes.begin(), _nodes.end(),
                           [&name](const auto& node) { return node.name == name; });
    if(it != _nodes.end()) {
      throw BadConstruction{"Function name already exists: " + name};
    }

    if(f.input_types().size() != inputs.size()) {
      throw BadConstruction{"Incorrect number of inputs for " + name + ": " +
                            std::to_string(inputs.size())};
    }

    // Check if moved inputs have been moved already
    boost::for_each(boost::combine(inputs, f.input_types()), [this](const auto& tuple) {
      auto term = boost::get<0>(tuple);
      const graph::Node& node = _nodes[term.node_id];
      bool ref_input = boost::get<1>(tuple).is_ref();

      if(ref_input || !node.no_copy)
        return;

      auto it = boost::find_if(node.outputs,
                               [](const graph::Edge& Edge) { return Edge.pb == PassBy::move; });

      if(it != node.outputs.end()) {
        throw BadConstruction{"Output can only be moved once"};
      }
    });

    // Check types match
    boost::for_each(boost::combine(inputs, f.input_types()), [this](const auto& tuple) {
      auto term = boost::get<0>(tuple);
      auto output_type = _nodes[term.node_id].types()[term.arg_idx];
      auto input_type = boost::get<1>(tuple);

      if(output_type != input_type)
        throw BadConstruction{"Types don't match"};
    });

    // Add Edges
    boost::for_each(
        boost::combine(inputs, f.input_types()) | boost::adaptors::indexed(),
        [this](const auto& ele) {
          auto src_term = boost::get<0>(ele.value());
          graph::Node& node = _nodes[src_term.node_id];
          bool ref_input = boost::get<1>(ele.value()).is_ref();

          PassBy pb = ref_input ? PassBy::ref : (node.no_copy ? PassBy::move : PassBy::copy);

          node.outputs.push_back(graph::Edge{
              src_term, graph::Term{static_cast<int>(_nodes.size()), static_cast<int>(ele.index())},
              pb});
        });

    _nodes.emplace_back(graph::FuncNode{std::move(f)}, std::move(name), std::move(output_names));
    return *this;
  }

  template <typename... Ts>
  friend ConstructingGraph<std::tuple<Ts...>> make_graph(std::array<std::string, sizeof...(Ts)> names);
};

template <typename... Inputs>
ConstructingGraph<std::tuple<Inputs...>> make_graph(std::array<std::string, sizeof...(Inputs)> names) {
  return ConstructingGraph<std::tuple<Inputs...>>(std::move(names), {make_type<Inputs>()...});
}

} // namespace anyf

#endif