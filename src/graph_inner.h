#pragma once

#include "anyf/graph.h"

namespace anyf {

struct Iterm {
  int node_id = 0;
  int port = 0;
  bool value = false;

  KNOT_ORDERED(Iterm);
};

struct ValueForward {
  std::vector<Iterm> terms;
  int copy_end = 0;
  int move_end = 0;

  KNOT_ORDERED(ValueForward);
};

struct IfExpr {
  FunctionGraph if_branch;
  FunctionGraph else_branch;
};

struct WhileExpr {
  FunctionGraph body;
};

using Expr = std::variant<std::shared_ptr<const AnyFunction>, IfExpr, WhileExpr>;

struct FunctionGraph::State {
  std::vector<TypeProperties> input_types;
  std::vector<TypeID> output_types;

  std::vector<std::vector<ValueForward>> owned_fwds;
  std::vector<ValueForward> input_borrowed_fwds;

  std::vector<std::pair<int, int>> input_counts;
  std::vector<Expr> exprs;
};

inline int num_outputs(const Expr& e) {
  return int(std::visit(Overloaded{[](const std::shared_ptr<const AnyFunction>& f) { return f->output_types().size(); },
                                   [](const WhileExpr& e) { return e.body.state->output_types.size() - 1; },
                                   [](const IfExpr& e) { return e.if_branch.state->output_types.size(); }},
                        e));
}

} // namespace anyf
