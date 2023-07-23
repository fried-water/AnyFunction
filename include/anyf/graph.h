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

struct Oterm {
  int node_id = 0;
  int port = 0;
  bool value = false;

  KNOT_ORDERED(Oterm);
};

struct FunctionGraph {
  struct State;
  std::shared_ptr<const State> state;

  ~FunctionGraph();

  Span<TypeProperties> input_types() const;
  Span<TypeID> output_types() const;
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

  tl::expected<std::vector<Oterm>, GraphError>
  add_functional(Span<TypeProperties>, std::vector<TypeID>, Oterm fn, Span<Oterm>);

  tl::expected<std::vector<Oterm>, GraphError> add_if(FunctionGraph, FunctionGraph, Span<Oterm>);
  tl::expected<std::vector<Oterm>, GraphError> add_select(Oterm cond, Span<Oterm> if_, Span<Oterm> else_);
  tl::expected<std::vector<Oterm>, GraphError> add_while(FunctionGraph, Span<Oterm>);

  tl::expected<FunctionGraph, GraphError> finalize(Span<Oterm>) &&;

  friend std::tuple<ConstructingGraph, std::vector<Oterm>> make_graph(std::vector<TypeProperties>);
};

std::tuple<ConstructingGraph, std::vector<Oterm>> make_graph(std::vector<TypeProperties> input_types);

FunctionGraph make_graph(AnyFunction);

} // namespace anyf
