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

using GraphError = std::variant<BadArity, BadType, AlreadyMoved, CannotCopy>;

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

struct FunctionGraph {
  std::vector<TypeProperties> input_types;
  std::vector<TypeID> output_types;

  std::vector<std::vector<ValueForward>> owned_fwds;
  std::vector<ValueForward> input_borrowed_fwds;

  std::vector<std::pair<int, int>> input_counts;
  std::vector<std::shared_ptr<const AnyFunction>> functions;
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

  tl::expected<std::vector<Oterm>, GraphError> add(AnyFunction, Span<Oterm> inputs);
  tl::expected<std::vector<Oterm>, GraphError> add(const FunctionGraph&, Span<Oterm> inputs);

  tl::expected<FunctionGraph, GraphError> finalize(Span<Oterm> outputs) &&;

  friend std::tuple<ConstructingGraph, std::vector<Oterm>> make_graph(std::vector<TypeProperties>);
};

std::tuple<ConstructingGraph, std::vector<Oterm>> make_graph(std::vector<TypeProperties> input_types);

} // namespace anyf
