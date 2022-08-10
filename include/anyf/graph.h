#pragma once

#include "anyf/any_function.h"
#include "anyf/traits.h"
#include "anyf/util.h"
#include "knot/core.h"

#include <memory>
#include <optional>
#include <unordered_map>
#include <variant>
#include <vector>

#include "tl/expected.hpp"

namespace anyf {

using NodeId = int;
using Port = int;

struct Term {
  NodeId node_id;
  Port port;

  KNOT_ORDERED(Term);
};

struct Edge {
  Port src_port;
  Term dst;

  KNOT_ORDERED(Edge);
};

struct TermUsage {
  int values = {};
  int borrows = {};
};

struct Node {
  std::variant<AnyFunction, std::vector<TypeProperties>, std::vector<TypeID>> func = std::vector<TypeProperties>{};
  std::vector<Edge> outputs = {};
};

struct BadArity {
  int expected = 0;
  int given = 0;

  KNOT_ORDERED(BadArity);
};

struct BadType {
  int index = 0;
  TypeID expected;
  TypeID given;

  KNOT_ORDERED(BadType);
};

struct AlreadyMoved {
  int index = 0;
  TypeID type;

  KNOT_ORDERED(AlreadyMoved);
};

struct CannotCopy {
  int index = 0;
  TypeID type;

  KNOT_ORDERED(CannotCopy);
};

inline std::string msg(const BadArity& e) {
  return fmt::format("Expected {} arguments, given {}", e.expected, e.given);
}

inline std::string msg(const BadType& e) { return fmt::format("Incorrect type for argument {}", e.index); }
inline std::string msg(const AlreadyMoved& e) { return fmt::format("Value for argument {} already moved", e.index); }
inline std::string msg(const CannotCopy& e) { return fmt::format("Input argument {} cannot be copied moved", e.index); }

using GraphError = std::variant<BadArity, BadType, AlreadyMoved, CannotCopy>;

inline std::vector<Term> make_terms(NodeId node, int size) {
  std::vector<Term> terms;
  for(int i = 0; i < size; i++) {
    terms.push_back({node, i});
  }
  return terms;
}

inline TypeProperties input_type(const std::vector<Node>& nodes, Term t) {
  return std::visit(Overloaded{[&](const std::vector<TypeProperties>& types) { return types[t.port]; },
                               [&](const std::vector<TypeID>& types) {
                                 return TypeProperties{types[t.port], true};
                               },
                               [&](const AnyFunction& f) { return f.input_types()[t.port]; }},
                    nodes[t.node_id].func);
}

inline TypeProperties output_type(const std::vector<Node>& nodes, Term t) {
  check(t.node_id < nodes.size(), "node {} out of range ({})", t.node_id, nodes.size());
  return std::visit(Overloaded{[&](const std::vector<TypeProperties>& types) { return types[t.port]; },
                               [&](const std::vector<TypeID>& types) {
                                 return TypeProperties{types[t.port], true};
                               },
                               [&](const AnyFunction& f) {
                                 return TypeProperties{f.output_types()[t.port], true};
                               }},
                    nodes[t.node_id].func);
}

using FunctionGraph = std::vector<Node>;

inline const std::vector<TypeProperties>& input_types(const FunctionGraph& g) {
  return std::get<std::vector<TypeProperties>>(g.front().func);
}

inline const std::vector<TypeID>& output_types(const FunctionGraph& g) {
  return std::get<std::vector<TypeID>>(g.back().func);
}

class ConstructingGraph {
public:
  ConstructingGraph() = default;

  explicit ConstructingGraph(std::vector<TypeProperties> inputs) { _nodes.push_back({std::move(inputs)}); }

  tl::expected<std::vector<Term>, GraphError> add(AnyFunction f, Span<Term> inputs) {
    const int num_outputs = static_cast<int>(f.output_types().size());

    const auto check_result = check_types(f.input_types(), inputs);

    if(!check_result) {
      return tl::unexpected{check_result.error()};
    }

    add_edges(inputs);
    _nodes.push_back({std::move(f)});

    return make_terms(static_cast<int>(_nodes.size() - 1), num_outputs);
  }

  tl::expected<std::vector<Term>, GraphError> add(const FunctionGraph& f, Span<Term> inputs) {
    check(f.size() >= 2, "F must have atleast 2 nodes");
    const auto check_result = check_types(input_types(f), inputs);

    if(!check_result) {
      return tl::unexpected{check_result.error()};
    }

    std::vector<std::vector<Term>> node_inputs(f.size() - 2);
    for(int i = 0; i < static_cast<int>(f.size() - 2); i++) {
      node_inputs[i].resize(std::get<AnyFunction>(f[i + 1].func).input_types().size());
    }

    std::vector<Term> outputs(output_types(f).size());

    const int initial_node_offset = static_cast<int>(_nodes.size()) - 1;

    for(int i = 0; i < static_cast<int>(f.size() - 1); i++) {
      for(const auto& [src_port, dst] : f[i].outputs) {
        const Term src_term{initial_node_offset + i, src_port};
        if(dst.node_id == f.size() - 1) {
          outputs[dst.port] = src_term;
        } else {
          node_inputs[dst.node_id - 1][dst.port] = i == 0 ? inputs[src_port] : src_term;
        }
      }

      if(i > 0) {
        add(std::get<AnyFunction>(f[i].func), std::move(node_inputs[i - 1]));
      }
    }

    return outputs;
  }

  tl::expected<FunctionGraph, GraphError> finalize(Span<Term> outputs) && {
    std::vector<TypeID> types;
    types.reserve(outputs.size());
    std::transform(
      outputs.begin(), outputs.end(), std::back_inserter(types), [&](Term t) { return output_type(_nodes, t).id; });

    std::vector<TypeProperties> props;
    props.reserve(outputs.size());
    std::transform(types.begin(), types.end(), std::back_inserter(props), [](TypeID t) {
      return TypeProperties{t, true};
    });

    const auto check_result = check_types(props, outputs);

    if(!check_result) {
      return tl::unexpected{check_result.error()};
    }

    add_edges(outputs);
    _nodes.push_back({std::move(types)});

    for(auto& [func, outputs] : _nodes) {
      std::sort(outputs.begin(), outputs.end());
    }

    _usage.clear();
    return std::move(_nodes);
  }

  TypeProperties type(Term t) const { return output_type(_nodes, t); }

private:
  tl::expected<std::monostate, GraphError> check_types(const std::vector<TypeProperties>& expected_types,
                                                       Span<Term> inputs) const {
    if(inputs.size() != expected_types.size()) {
      return tl::unexpected{BadArity{int(expected_types.size()), int(inputs.size())}};
    }

    // inputs moved in this invocation alone
    std::vector<Term> moved_inputs;

    for(int i = 0; i < inputs.ssize(); i++) {
      const auto& term = inputs[i];
      const TypeProperties input_type = expected_types[i];
      const TypeProperties given_type = output_type(_nodes, term);

      if(given_type.id != input_type.id) {
        return tl::unexpected{BadType{i, input_type.id, given_type.id}};
      } else if(!input_type.value || is_copyable(input_type.id)) {
        // always fine
      } else if(!given_type.value) {
        return tl::unexpected{CannotCopy{term.port, input_type.id}};
      } else {
        const auto usage_it = _usage.find(term);
        const auto cur_it = std::find(moved_inputs.begin(), moved_inputs.end(), term);
        if(cur_it != moved_inputs.end() || (usage_it != _usage.end() && usage_it->second.values > 0)) {
          return tl::unexpected{AlreadyMoved{i, input_type.id}};
        } else {
          moved_inputs.push_back(term);
        }
      }
    }

    return std::monostate{};
  }

  void add_edges(Span<Term> inputs) {
    for(int i = 0; i < inputs.ssize(); i++) {
      const auto term = inputs[i];

      if(output_type(_nodes, term).value) {
        _usage[term].values++;
      } else {
        _usage[term].borrows++;
      }

      _nodes[term.node_id].outputs.push_back({term.port, {static_cast<int>(_nodes.size()), i}});
    }
  }

  std::vector<Node> _nodes;
  std::unordered_map<Term, TermUsage, knot::Hash> _usage;
};

inline std::tuple<ConstructingGraph, std::vector<Term>> make_graph(std::vector<TypeProperties> types) {
  const int num_inputs = static_cast<int>(types.size());
  return {ConstructingGraph{std::move(types)}, make_terms(0, num_inputs)};
}

} // namespace anyf
