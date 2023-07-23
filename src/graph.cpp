#include "anyf/util.h"
#include "graph_inner.h"

#include <unordered_map>

namespace anyf {

namespace {

struct OtermUsage {
  int values = {};
  int borrows = {};
};

} // namespace

struct ConstructingGraph::State {
  FunctionGraph::State g;
  std::unordered_map<Oterm, OtermUsage, knot::Hash> usage;
};

namespace {

std::vector<Oterm> make_oterms(int node, Span<TypeProperties> types) {
  std::vector<Oterm> terms;
  terms.reserve(types.size());

  int value_idx = 0;
  int borrow_idx = 0;

  for(const TypeProperties& t : types) {
    terms.push_back({node, t.value ? value_idx++ : borrow_idx++, t.value});
  }

  return terms;
}

std::vector<Oterm> make_oterms(int node, int size) {
  std::vector<Oterm> terms;
  terms.reserve(size);
  for(int i = 0; i < size; i++) {
    terms.push_back({node, i, true});
  }
  return terms;
}

std::pair<int, int> counts(Span<TypeProperties> types) {
  const auto values = std::count_if(types.begin(), types.end(), [](auto t) { return t.value; });
  return {int(values), int(types.size() - values)};
}

TypeProperties type(Span<TypeProperties> types, int port, bool is_value) {
  auto it = std::find_if(types.begin(), types.end(), [&](auto t) { return t.value == is_value; });
  while(port-- > 0 && it != types.end()) {
    it = std::find_if(it + 1, types.end(), [&](auto t) { return t.value == is_value; });
  }
  assert(it != types.end());
  return *it;
}

TypeProperties type(const FunctionGraph::State& g, Oterm t) {
  if(t.node_id == 0) {
    return type(g.input_types, t.port, t.value);
  } else if(t.node_id <= g.exprs.size()) {
    assert(t.value);
    return std::visit(Overloaded{[&](const std::shared_ptr<const AnyFunction>& f) {
                                   return TypeProperties{f->output_types()[t.port], true};
                                 },
                                 [&](const SExpr& i) {
                                   return TypeProperties{i.types[t.port], true};
                                 },
                                 [&](const IfExpr& i) {
                                   return TypeProperties{i.if_branch.state->output_types[t.port], true};
                                 },
                                 [&](const SelectExpr& s) { return s.types[t.port]; },
                                 [&](const WhileExpr& w) {
                                   return TypeProperties{w.body.state->output_types[t.port + 1], true};
                                 }},
                      g.exprs[t.node_id - 1]);
  } else {
    assert(t.value);
    return {g.output_types[t.port], true};
  }
}

ValueForward& fwd_of(FunctionGraph::State& g, Oterm t) {
  if(t.value) {
    return g.owned_fwds[t.node_id][t.port];
  } else {
    assert(t.node_id == 0);
    return g.input_borrowed_fwds[t.port];
  }
}

std::optional<GraphError> check_types(const FunctionGraph::State& g,
                                      const std::unordered_map<Oterm, OtermUsage, knot::Hash>& usage,
                                      Span<TypeProperties> expected_types,
                                      Span<Oterm> inputs) {
  if(inputs.size() != expected_types.size()) {
    return GraphError{BadArity{int(expected_types.size()), int(inputs.size())}};
  }

  // inputs moved in this invocation alone
  std::vector<Oterm> moved_inputs;

  for(int i = 0; i < inputs.ssize(); i++) {
    const Oterm oterm = inputs[i];
    const TypeProperties input_type = expected_types[i];
    const TypeProperties given_type = type(g, oterm);

    if(given_type.id != input_type.id) {
      return GraphError{BadType{i, input_type.id, given_type.id}};
    } else if(!input_type.value || is_copyable(input_type.id)) {
      // always fine
    } else if(!given_type.value) {
      return GraphError{CannotCopy{oterm.port, input_type.id}};
    } else {
      const auto usage_it = usage.find(oterm);
      const auto cur_it = std::find(moved_inputs.begin(), moved_inputs.end(), oterm);
      if(cur_it != moved_inputs.end() || (usage_it != usage.end() && usage_it->second.values > 0)) {
        return GraphError{AlreadyMoved{i, input_type.id}};
      } else {
        moved_inputs.push_back(oterm);
      }
    }
  }

  return {};
}

void add_edges(FunctionGraph::State& g,
               std::unordered_map<Oterm, OtermUsage, knot::Hash>& usage,
               Span<Oterm> inputs,
               Span<TypeProperties> input_types) {
  assert(inputs.size() == input_types.size());

  int value_idx = 0;
  int borrow_idx = 0;

  for(int i = 0; i < inputs.ssize(); i++) {
    const Oterm oterm = inputs[i];
    const Iterm iterm = {int(g.exprs.size()), input_types[i].value ? value_idx++ : borrow_idx++, input_types[i].value};

    if(input_types[i].value) {
      usage[oterm].values++;
    } else {
      usage[oterm].borrows++;
    }

    fwd_of(g, oterm).terms.push_back(iterm);
  }
}

ValueForward fixup_fwd(TypeProperties type, ValueForward fwd) {
  const auto ref_begin = std::stable_partition(fwd.terms.begin(), fwd.terms.end(), [&](Iterm t) { return t.value; });

  fwd.move_end = static_cast<int>(std::distance(fwd.terms.begin(), ref_begin));
  fwd.copy_end = type.value ? std::max(0, fwd.move_end - 1) : fwd.move_end;

  if(ref_begin != fwd.terms.end() && is_copyable(type.id) && fwd.copy_end != fwd.move_end) {
    fwd.copy_end++;
  }

  return fwd;
}

tl::expected<std::vector<Oterm>, GraphError> add_internal(FunctionGraph::State& g,
                                                          std::unordered_map<Oterm, OtermUsage, knot::Hash>& usage,
                                                          Expr expr,
                                                          Span<Oterm> inputs,
                                                          Span<TypeProperties> input_types) {
  if(const auto check_failure = check_types(g, usage, input_types, inputs); check_failure) {
    return tl::unexpected{*check_failure};
  }

  add_edges(g, usage, inputs, input_types);

  const int output_count = num_outputs(expr);
  g.input_counts.push_back(counts(input_types));
  g.exprs.push_back(std::move(expr));
  g.owned_fwds.push_back(std::vector<ValueForward>(output_count));

  return make_oterms(int(g.exprs.size()), output_count);
}

} // namespace

TypeProperties ConstructingGraph::type(Oterm oterm) { return anyf::type(_state->g, oterm); }

tl::expected<std::vector<Oterm>, GraphError> ConstructingGraph::add(AnyFunction f, Span<Oterm> inputs) {
  const auto shared_f = std::make_shared<const AnyFunction>(std::move(f));
  return add_internal(_state->g, _state->usage, shared_f, inputs, shared_f->input_types());
}

tl::expected<std::vector<Oterm>, GraphError> ConstructingGraph::add(const FunctionGraph& g_outer, Span<Oterm> inputs) {
  const FunctionGraph::State& g = *g_outer.state;

  if(const auto check_result = check_types(_state->g, _state->usage, g.input_types, inputs); check_result) {
    return tl::unexpected{*check_result};
  }

  const int offset = int(_state->g.owned_fwds.size()) - 1;

  std::vector<Oterm> outputs(g.output_types.size());

  const auto process_fwds = [&](Oterm fanin, const auto& input_terms, auto& output_terms) {
    for(Iterm fanout : input_terms) {
      if(fanout.node_id == g.exprs.size()) {
        outputs[fanout.port] = fanin;
      } else {
        output_terms.push_back({fanout.node_id + offset, fanout.port, fanout.value});
      }
    }
  };

  int value_idx = 0;
  int borrow_idx = 0;

  for(int i = 0; i < inputs.size(); i++) {
    const ValueForward& inner_fwd =
      g.input_types[i].value ? g.owned_fwds[0][value_idx++] : g.input_borrowed_fwds[borrow_idx++];
    process_fwds(inputs[i], inner_fwd.terms, fwd_of(_state->g, inputs[i]).terms);
  }

  for(int n = 0; n < g.exprs.size(); n++) {
    _state->g.input_counts.push_back(g.input_counts[n]);
    _state->g.exprs.push_back(g.exprs[n]);
    _state->g.owned_fwds.push_back(std::vector<ValueForward>(g.owned_fwds[n + 1].size()));

    for(int p = 0; p < g.owned_fwds[n + 1].size(); p++) {
      process_fwds({n + 1 + offset, p, true}, g.owned_fwds[n + 1][p].terms, _state->g.owned_fwds.back()[p].terms);
    }
  }

  return outputs;
}

tl::expected<std::vector<Oterm>, GraphError> ConstructingGraph::add_functional(Span<TypeProperties> fn_input_types,
                                                                               std::vector<TypeID> outputs,
                                                                               Oterm fn,
                                                                               Span<Oterm> fn_inputs) {
  // TODO: find a way to type check fn arguments and derive fn types

  std::vector<Oterm> inputs;
  inputs.reserve(fn_inputs.size() + 1);
  inputs.push_back(fn);
  inputs.insert(inputs.end(), fn_inputs.begin(), fn_inputs.end());

  std::vector<TypeProperties> input_types;
  input_types.reserve(fn_input_types.size() + 1);
  input_types.push_back({type_id<FunctionGraph>(), true});
  std::copy(fn_input_types.begin(), fn_input_types.end(), std::back_inserter(input_types));

  return add_internal(_state->g, _state->usage, SExpr{std::move(outputs)}, inputs, input_types);
}

tl::expected<std::vector<Oterm>, GraphError>
ConstructingGraph::add_if(FunctionGraph if_branch, FunctionGraph else_branch, Span<Oterm> inputs) {
  if(if_branch.state->input_types != else_branch.state->input_types ||
     if_branch.state->output_types != else_branch.state->output_types) {
    return tl::unexpected{GraphError{MismatchedBranchTypes{}}};
  }

  std::vector<TypeProperties> input_types;
  input_types.reserve(if_branch.state->input_types.size() + 1);
  input_types.push_back(TypeProperties{type_id<bool>(), true});
  input_types.insert(input_types.end(), if_branch.state->input_types.begin(), if_branch.state->input_types.end());

  return add_internal(
    _state->g, _state->usage, IfExpr{std::move(if_branch), std::move(else_branch)}, inputs, input_types);
}

tl::expected<std::vector<Oterm>, GraphError>
ConstructingGraph::add_select(Oterm cond, Span<Oterm> if_branch, Span<Oterm> else_branch) {
  if(if_branch.size() != else_branch.size()) {
    return tl::unexpected{GraphError{MismatchedBranchTypes{}}};
  }

  std::vector<TypeProperties> types;
  types.reserve(if_branch.size());
  std::transform(if_branch.begin(), if_branch.end(), std::back_inserter(types), [&](Oterm o) { return type(o); });

  for(size_t i = 0; i < if_branch.size(); i++) {
    if(types[i] != type(else_branch[i])) {
      return tl::unexpected{GraphError{MismatchedBranchTypes{}}};
    }
  }

  std::vector<TypeProperties> input_types;
  input_types.reserve(if_branch.size() * 2 + 1);

  input_types.push_back({type_id<bool>(), true});
  std::transform(types.begin(),
                 types.end(),
                 std::back_inserter(input_types),
                 // TODO: allow select to return borrows
                 [&](TypeProperties t) {
                   return TypeProperties{t.id, true};
                 });
  input_types.insert(input_types.end(), input_types.begin() + 1, input_types.end());

  std::vector<Oterm> inputs;
  inputs.reserve(if_branch.size() * 2 + 1);
  inputs.push_back(cond);
  inputs.insert(inputs.end(), if_branch.begin(), if_branch.end());
  inputs.insert(inputs.end(), else_branch.begin(), else_branch.end());

  return add_internal(_state->g, _state->usage, SelectExpr{std::move(types)}, inputs, input_types);
}

tl::expected<std::vector<Oterm>, GraphError> ConstructingGraph::add_while(FunctionGraph g, Span<Oterm> inputs) {
  std::vector<TypeProperties> input_types;
  input_types.reserve(g.state->input_types.size() + 1);
  input_types.push_back(TypeProperties{type_id<bool>(), true});
  input_types.insert(input_types.end(), g.state->input_types.begin(), g.state->input_types.end());

  return add_internal(_state->g, _state->usage, WhileExpr{std::move(g)}, inputs, input_types);
}

tl::expected<FunctionGraph, GraphError> ConstructingGraph::finalize(Span<Oterm> outputs) && {
  assert(_state->g.owned_fwds.size() > 0);

  _state->g.output_types.reserve(outputs.size());
  std::transform(outputs.begin(), outputs.end(), std::back_inserter(_state->g.output_types), [&](Oterm t) {
    return anyf::type(_state->g, t).id;
  });

  std::vector<TypeProperties> props;
  props.reserve(outputs.size());
  std::transform(_state->g.output_types.begin(), _state->g.output_types.end(), std::back_inserter(props), [](TypeID t) {
    return TypeProperties{t, true};
  });

  if(const auto check_result = check_types(_state->g, _state->usage, props, outputs); check_result) {
    return tl::unexpected{*check_result};
  }

  add_edges(_state->g, _state->usage, outputs, props);

  for(int i = 0; i < int(_state->g.owned_fwds.size()); i++) {
    for(int p = 0; p < int(_state->g.owned_fwds[i].size()); p++) {
      _state->g.owned_fwds[i][p] =
        fixup_fwd(anyf::type(_state->g, Oterm{i, p, true}), std::move(_state->g.owned_fwds[i][p]));
    }
  }

  for(int i = 0; i < int(_state->g.input_borrowed_fwds.size()); i++) {
    _state->g.input_borrowed_fwds[i] =
      fixup_fwd(anyf::type(_state->g, Oterm{0, i, false}), std::move(_state->g.input_borrowed_fwds[i]));
  }

  return FunctionGraph{std::make_shared<const FunctionGraph::State>(std::move(_state->g))};
}

std::tuple<ConstructingGraph, std::vector<Oterm>> make_graph(std::vector<TypeProperties> types) {
  std::vector<Oterm> terms = make_oterms(0, types);
  return {ConstructingGraph{std::move(types)}, std::move(terms)};
}

FunctionGraph make_graph(AnyFunction f) {
  auto [cg, inputs] = make_graph(f.input_types());
  return *std::move(cg).finalize(*cg.add(std::move(f), inputs));
}

std::string msg(const GraphError& e) {
  return std::visit(
    Overloaded{
      [](const BadArity& e) { return fmt::format("Expected {} arguments, given {}", e.expected, e.given); },
      [](const BadType& e) { return fmt::format("Incorrect type for argument {}", e.index); },
      [](const AlreadyMoved& e) { return fmt::format("Value for argument {} already moved", e.index); },
      [](const CannotCopy& e) { return fmt::format("Input argument {} cannot be copied moved", e.index); },
      [](const MismatchedBranchTypes&) { return fmt::format("If and else branch graphs have different types"); }},
    e);
}

ConstructingGraph::ConstructingGraph(std::vector<TypeProperties> _input_types)
    : _state(std::make_unique<State>(State{FunctionGraph::State{std::move(_input_types)}})) {
  auto [value_count, borrow_count] = counts(_state->g.input_types);
  _state->g.owned_fwds.push_back(std::vector<ValueForward>(value_count));
  _state->g.input_borrowed_fwds.resize(borrow_count);
}

ConstructingGraph::~ConstructingGraph() = default;

ConstructingGraph::ConstructingGraph(ConstructingGraph&&) = default;
ConstructingGraph& ConstructingGraph::operator=(ConstructingGraph&&) = default;

Span<TypeProperties> FunctionGraph::input_types() const { return state->input_types; }
Span<TypeID> FunctionGraph::output_types() const { return state->output_types; }

FunctionGraph::~FunctionGraph() = default;

} // namespace anyf
