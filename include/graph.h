#pragma once

#include "any_function.h"
#include "traits.h"
#include "util.h"

#include <memory>
#include <optional>
#include <unordered_map>
#include <variant>
#include <vector>

#include <iostream>

namespace anyf {

using NodeId = int;
using Port = int;

struct Term {
  NodeId node_id;
  Port port;
  friend bool operator==(const Term t1, const Term t2) {
    return std::tie(t1.node_id, t1.port) == std::tie(t2.node_id, t2.port);
  }
  friend bool operator<(const Term t1, const Term t2) {
    return std::tie(t1.node_id, t1.port) < std::tie(t2.node_id, t2.port);
  }
};

struct Edge {
  Port src_port;
  Term dst;
  friend bool operator==(const Edge e1, const Edge e2) {
    return std::tie(e1.src_port, e1.dst) == std::tie(e2.src_port, e2.dst);
  }
  friend bool operator<(const Edge e1, const Edge e2) {
    return std::tie(e1.src_port, e1.dst) < std::tie(e2.src_port, e2.dst);
  }
};

struct TermHash {
  size_t operator()(const Term& t) const { return t.node_id ^ t.port; }
};

struct TermUsage {
  int values = {};
  int borrows = {};
};

struct Node {
  std::variant<AnyFunction, std::vector<Type>> func = std::vector<Type>{};
  std::vector<Edge> outputs = {};
};

inline std::vector<Term> make_terms(NodeId node, int size) {
  std::vector<Term> terms;
  for(int i = 0; i < size; i++) {
    terms.push_back({node, i});
  }
  return terms;
}

inline Type input_type(const std::vector<Node>& nodes, Term t) {
  return std::visit(Overloaded{[&](const std::vector<Type>& types) { return types[t.port]; },
                               [&](const AnyFunction& f) { return f.input_types()[t.port]; }},
                    nodes[t.node_id].func);
}

inline Type output_type(const std::vector<Node>& nodes, Term t) {
  return std::visit(Overloaded{[&](const std::vector<Type>& types) { return types[t.port]; },
                               [&](const AnyFunction& f) { return f.output_types()[t.port]; }},
                    nodes[t.node_id].func);
}

using FunctionGraph = std::vector<Node>;

class ConstructingGraph {
public:
  ConstructingGraph() = default;

  explicit ConstructingGraph(std::vector<Type> inputs) { _nodes.push_back({std::move(inputs)}); }

  std::vector<Term> add(AnyFunction f, Span<Term> inputs) {
    const int num_outputs = static_cast<int>(f.output_types().size());

    check_types(f.input_types(), inputs);
    add_edges(inputs);
    _nodes.push_back({std::move(f)});

    return make_terms(static_cast<int>(_nodes.size() - 1), num_outputs);
    ;
  }

  std::vector<Term> add(const FunctionGraph& f, Span<Term> inputs) {
    check(f.size() >= 2, "F must have atleast 2 nodes");
    check_types(std::get<std::vector<Type>>(f[0].func), inputs);

    std::vector<std::vector<Term>> node_inputs(f.size() - 2);
    for(int i = 0; i < static_cast<int>(f.size() - 2); i++) {
      node_inputs[i].resize(std::get<AnyFunction>(f[i + 1].func).input_types().size());
    }

    std::vector<Term> outputs(std::get<std::vector<Type>>(f.back().func).size());

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

  friend FunctionGraph finalize(ConstructingGraph, Span<Term>);

private:
  void check_types(const std::vector<Type>& expected_types, Span<Term> inputs) const {
    check(inputs.size() == expected_types.size(), "Function expected {} arguments, given {}",
          expected_types.size(), inputs.size());

    for(int i = 0; i < inputs.ssize(); i++) {
      const auto& term = inputs[i];
      const Type input_type = expected_types[i];

      check(output_type(_nodes, term).type_id() == input_type.type_id(),
            "Type mismatch at argument {}", i);

      if(input_type.is_ref() || input_type.is_copy_constructible()) {
        // Always fine
      } else if(input_type.is_move_constructible()) {
        const auto it = _usage.find(term);
        check(it == _usage.end() || it->second.values == 0, "Argument {} already moved", i);
      } else {
        error("Argument {} cannot be moved or copied", i);
      }
    }
  }

  void add_edges(Span<Term> inputs) {
    for(int i = 0; i < inputs.ssize(); i++) {
      const auto term = inputs[i];

      if(output_type(_nodes, term).is_ref()) {
        _usage[term].borrows++;
      } else {
        _usage[term].values++;
      }

      _nodes[term.node_id].outputs.push_back({term.port, {static_cast<int>(_nodes.size()), i}});
    }
  }

  std::vector<Node> _nodes;
  std::unordered_map<Term, TermUsage, TermHash> _usage;
};

inline std::tuple<ConstructingGraph, std::vector<Term>> make_graph(std::vector<Type> types) {
  const int num_inputs = static_cast<int>(types.size());
  return {ConstructingGraph{std::move(types)}, make_terms(0, num_inputs)};
}

inline FunctionGraph finalize(ConstructingGraph cg, Span<Term> outputs) {
  std::vector<Type> types;

  for(int i = 0; i < outputs.ssize(); i++) {
    types.push_back(output_type(cg._nodes, outputs[i]));

    check(!types.back().is_ref(), "Cannot return a borrowed value for output {}", i);

    check(types.back().is_copy_constructible() ||
              (cg._usage.at(outputs[i]).values == 0 && types.back().is_move_constructible()),
          "Cannot return output {} since its not copy constructible or its already moved");

    cg._nodes[outputs[i].node_id].outputs.push_back(
        {outputs[i].port, {static_cast<int>(cg._nodes.size()), i}});
  }

  cg._nodes.push_back({std::move(types)});

  for(auto& [func, outputs] : cg._nodes) {
    std::sort(outputs.begin(), outputs.end());
  }

  return std::move(cg._nodes);
}

} // namespace anyf
