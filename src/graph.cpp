#include "anyf/graph.h"

#include "anyf/util.h"

#include <unordered_map>

namespace anyf {

namespace {

struct OtermUsage {
  int values = {};
  int borrows = {};
};

} // namespace

struct ConstructingGraph::State {
  FunctionGraph g;
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
  const auto is_value = [](TypeProperties t) { return t.value; };
  return {int(std::count_if(types.begin(), types.end(), is_value)),
          int(std::count_if(types.begin(), types.end(), std::not_fn(is_value)))};
}

TypeProperties type(Span<TypeProperties> types, int port, bool is_value) {
  auto it = std::find_if(types.begin(), types.end(), [&](auto t) { return t.value == is_value; });
  while(port-- > 0 && it != types.end()) {
    it = std::find_if(it + 1, types.end(), [&](auto t) { return t.value == is_value; });
  }
  assert(it != types.end());
  return *it;
}

TypeProperties type(const FunctionGraph& g, Oterm t) {
  if(t.node_id == 0) {
    return type(g.input_types, t.port, t.value);
  } else if(t.node_id <= g.exprs.size()) {
    assert(t.value);
    return std::visit(Overloaded{
                        [&](const std::shared_ptr<const AnyFunction>& f) {
                          return TypeProperties{f->output_types()[t.port], true};
                        },
                        [&](const WhileExpr& w) {
                          return TypeProperties{w.body->output_types[t.port + 1], true};
                        },
                      },
                      g.exprs[t.node_id - 1]);
  } else {
    assert(t.value);
    return {g.output_types[t.port], true};
  }
}

template <typename G>
auto& fwd_of(G& g, Oterm t) {
  if(t.value) {
    return g.owned_fwds[t.node_id][t.port];
  } else {
    assert(t.node_id == 0);
    return g.input_borrowed_fwds[t.port];
  }
}

std::optional<GraphError> check_types(const FunctionGraph& g,
                                      const std::unordered_map<Oterm, OtermUsage, knot::Hash>& usage,
                                      const std::vector<TypeProperties>& expected_types,
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

void add_edges(FunctionGraph& g,
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

} // namespace

tl::expected<std::vector<Oterm>, GraphError> ConstructingGraph::add(AnyFunction f, Span<Oterm> inputs) {
  if(const auto check_failure = check_types(_state->g, _state->usage, f.input_types(), inputs); check_failure) {
    return tl::unexpected{*check_failure};
  }

  add_edges(_state->g, _state->usage, inputs, f.input_types());

  const int num_outputs = int(f.output_types().size());
  _state->g.input_counts.push_back(counts(f.input_types()));
  _state->g.exprs.push_back(std::make_shared<const AnyFunction>(std::move(f)));
  _state->g.owned_fwds.push_back(std::vector<ValueForward>(num_outputs));

  return make_oterms(int(_state->g.exprs.size()), num_outputs);
}

tl::expected<std::vector<Oterm>, GraphError> ConstructingGraph::add(const FunctionGraph& f, Span<Oterm> inputs) {
  if(const auto check_result = check_types(_state->g, _state->usage, f.input_types, inputs); check_result) {
    return tl::unexpected{*check_result};
  }

  const int offset = int(_state->g.owned_fwds.size()) - 1;

  std::vector<Oterm> outputs(f.output_types.size());

  const auto process_fwds = [&](Oterm fanin, const auto& input_terms, auto& output_terms) {
    for(Iterm fanout : input_terms) {
      if(fanout.node_id == f.exprs.size()) {
        outputs[fanout.port] = fanin;
      } else {
        output_terms.push_back({fanout.node_id + offset, fanout.port, fanout.value});
      }
    }
  };

  int value_idx = 0;
  int borrow_idx = 0;

  for(const Oterm input : inputs) {
    const ValueForward& inner_fwd = input.value ? f.owned_fwds[0][value_idx++] : f.input_borrowed_fwds[borrow_idx++];

    process_fwds(input, inner_fwd.terms, fwd_of(_state->g, input).terms);
  }

  for(int n = 0; n < f.exprs.size(); n++) {
    _state->g.input_counts.push_back(f.input_counts[n]);
    _state->g.exprs.push_back(f.exprs[n]);
    _state->g.owned_fwds.push_back(std::vector<ValueForward>(f.owned_fwds[n + 1].size()));

    for(int p = 0; p < f.owned_fwds[n + 1].size(); p++) {
      process_fwds({n + 1 + offset, p, true}, f.owned_fwds[n + 1][p].terms, _state->g.owned_fwds.back()[p].terms);
    }
  }

  return outputs;
}

tl::expected<std::vector<Oterm>, GraphError> ConstructingGraph::add_while(const FunctionGraph& f, Span<Oterm> inputs) {
  std::vector<TypeProperties> input_types;
  input_types.reserve(f.input_types.size() + 1);
  input_types.push_back(TypeProperties{type_id<bool>(), true});
  input_types.insert(input_types.end(), f.input_types.begin(), f.input_types.end());

  if(const auto check_failure = check_types(_state->g, _state->usage, input_types, inputs); check_failure) {
    return tl::unexpected{*check_failure};
  }

  add_edges(_state->g, _state->usage, inputs, input_types);

  const int num_outputs = int(f.output_types.size() - 1);
  _state->g.input_counts.push_back(counts(input_types));
  _state->g.exprs.push_back(WhileExpr{std::make_shared<const FunctionGraph>(f)});
  _state->g.owned_fwds.push_back(std::vector<ValueForward>(num_outputs));

  return make_oterms(int(_state->g.exprs.size()), num_outputs);
}

tl::expected<FunctionGraph, GraphError> ConstructingGraph::finalize(Span<Oterm> outputs) && {
  assert(_state->g.owned_fwds.size() > 0);

  _state->g.output_types.reserve(outputs.size());
  std::transform(outputs.begin(), outputs.end(), std::back_inserter(_state->g.output_types), [&](Oterm t) {
    return type(_state->g, t).id;
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
      _state->g.owned_fwds[i][p] = fixup_fwd(type(_state->g, Oterm{i, p, true}), std::move(_state->g.owned_fwds[i][p]));
    }
  }

  for(int i = 0; i < int(_state->g.input_borrowed_fwds.size()); i++) {
    _state->g.input_borrowed_fwds[i] =
      fixup_fwd(type(_state->g, Oterm{0, i, false}), std::move(_state->g.input_borrowed_fwds[i]));
  }

  return std::move(_state->g);
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
    Overloaded{[](const BadArity& e) { return fmt::format("Expected {} arguments, given {}", e.expected, e.given); },
               [](const BadType& e) { return fmt::format("Incorrect type for argument {}", e.index); },
               [](const AlreadyMoved& e) { return fmt::format("Value for argument {} already moved", e.index); },
               [](const CannotCopy& e) { return fmt::format("Input argument {} cannot be copied moved", e.index); }},
    e);
}

ConstructingGraph::ConstructingGraph(std::vector<TypeProperties> _input_types)
    : _state(std::make_unique<State>(State{FunctionGraph{std::move(_input_types)}})) {
  auto [value_count, borrow_count] = counts(_state->g.input_types);
  _state->g.owned_fwds.push_back(std::vector<ValueForward>(value_count));
  _state->g.input_borrowed_fwds.resize(borrow_count);
}

ConstructingGraph::~ConstructingGraph() = default;

ConstructingGraph::ConstructingGraph(ConstructingGraph&&) = default;
ConstructingGraph& ConstructingGraph::operator=(ConstructingGraph&&) = default;

} // namespace anyf
