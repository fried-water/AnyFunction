#pragma once

#include "anyf/any_function.h"
#include "anyf/traits.h"

#include <knot/core.h>

#include <tl/expected.hpp>

#include <memory>
#include <variant>
#include <vector>

namespace anyf {

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

struct MismatchedBranchTypes {
  KNOT_ORDERED(MismatchedBranchTypes);
};

using GraphError = std::variant<BadArity, BadType, AlreadyMoved, CannotCopy, MismatchedBranchTypes>;

std::string msg(const GraphError& e);

struct Iterm {
  int node_id = 0;
  int port = 0;
  bool value = false;

  KNOT_ORDERED(Iterm);
};

struct Oterm {
  int node_id = 0;
  int port = 0;
  bool value = false;

  KNOT_ORDERED(Oterm);
};

struct ValueForward {
  std::vector<Iterm> terms;
  int copy_end = 0;
  int move_end = 0;

  KNOT_ORDERED(ValueForward);
};

struct IfExpr;
struct WhileExpr;

using Expr = std::variant<std::shared_ptr<const AnyFunction>, IfExpr, WhileExpr>;

struct FunctionGraph {
  std::vector<TypeProperties> input_types;
  std::vector<TypeID> output_types;

  std::vector<std::vector<ValueForward>> owned_fwds;
  std::vector<ValueForward> input_borrowed_fwds;

  std::vector<std::pair<int, int>> input_counts;
  std::vector<Expr> exprs;
};

struct IfExpr {
  std::shared_ptr<const FunctionGraph> if_branch;
  std::shared_ptr<const FunctionGraph> else_branch;
};

struct WhileExpr {
  std::shared_ptr<const FunctionGraph> body;
};

class ConstructingGraph {
  struct State;
  std::unique_ptr<State> _state;

  explicit ConstructingGraph(std::vector<TypeProperties>);

public:
  ConstructingGraph() = default;
  ~ConstructingGraph();

  ConstructingGraph(ConstructingGraph&&);
  ConstructingGraph& operator=(ConstructingGraph&&);

  TypeProperties type(Oterm);

  tl::expected<std::vector<Oterm>, GraphError> add(AnyFunction, Span<Oterm>);
  tl::expected<std::vector<Oterm>, GraphError> add(const FunctionGraph&, Span<Oterm>);

  tl::expected<std::vector<Oterm>, GraphError> add_if(const FunctionGraph&, const FunctionGraph&, Span<Oterm>);
  tl::expected<std::vector<Oterm>, GraphError> add_while(const FunctionGraph&, Span<Oterm>);

  tl::expected<FunctionGraph, GraphError> finalize(Span<Oterm>) &&;

  friend std::tuple<ConstructingGraph, std::vector<Oterm>> make_graph(std::vector<TypeProperties>);
};

std::tuple<ConstructingGraph, std::vector<Oterm>> make_graph(std::vector<TypeProperties> input_types);

FunctionGraph make_graph(AnyFunction);

int num_outputs(const Expr&);

} // namespace anyf
