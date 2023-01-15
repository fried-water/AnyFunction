#include "anyf/graph_execution.h"

#include <algorithm>
#include <atomic>
#include <memory>
#include <optional>

namespace anyf {

namespace {

struct InvocationBlock {
  std::atomic<int> ref_count;

  std::vector<Any> input_vals;
  std::vector<Any*> input_ptrs;
  std::vector<BorrowedFuture> borrowed_inputs;

  std::vector<Promise> promises;

  InvocationBlock(int num_inputs, std::vector<BorrowedFuture> borrowed_inputs)
      : ref_count(num_inputs)
      , input_vals(num_inputs)
      , input_ptrs(num_inputs)
      , borrowed_inputs(std::move(borrowed_inputs)) {}
};

struct InputState {
  std::vector<std::vector<Future>> inputs;
  std::vector<std::vector<BorrowedFuture>> borrowed_inputs;
};

std::vector<Future> invoke_async(Executor e,
                                 const std::shared_ptr<const AnyFunction>& func,
                                 std::vector<Future> inputs,
                                 std::vector<BorrowedFuture> borrowed_inputs) {
  const int num_inputs = int(func->input_types().size());
  const int num_outputs = int(func->output_types().size());

  auto block = std::make_shared<InvocationBlock>(num_inputs, std::move(borrowed_inputs));

  std::vector<Future> futures;

  block->promises.reserve(num_outputs);
  futures.reserve(num_outputs);

  for(size_t i = 0; i < num_outputs; i++) {
    auto [p, f] = make_promise_future(e);
    block->promises.push_back(std::move(p));
    futures.push_back(std::move(f));
  }

  const auto try_invoke = [=](auto& block) {
    if(decrement(block->ref_count) == 1) {
      auto results = (*func)(block->input_ptrs);

      // Drop reference to all borrowed inputs so they can be forwarded asap
      block->borrowed_inputs.clear();

      // Forward outputs
      for(size_t i = 0; i < results.size(); i++) {
        std::move(block->promises[i]).send(std::move(results[i]));
      }
    }
  };

  int owned_offset = 0;
  int borrowed_offset = 0;
  for(size_t i = 0; i < num_inputs; i++) {
    if(func->input_types()[i].value) {
      std::move(inputs[owned_offset++]).then([=](Any value) mutable {
        block->input_vals[i] = std::move(value);
        block->input_ptrs[i] = &block->input_vals[i];
        try_invoke(block);
      });
    } else {
      block->borrowed_inputs[borrowed_offset++].then([i, block, try_invoke](const Any& value) {
        block->input_ptrs[i] = const_cast<Any*>(&value);
        try_invoke(block);
      });
    }
  }

  return futures;
}

InputState propagate(InputState s, Executor e, const std::vector<ValueForward>& fwds, std::vector<Future> outputs) {
  assert(fwds.size() == outputs.size());

  for(int i = 0; i < int(fwds.size()); i++) {
    const auto& fwd = fwds[i];

    std::vector<Promise> copy_promises;
    copy_promises.reserve(fwd.copy_end);

    for(int i = 0; i < fwd.copy_end; i++) {
      auto [p, f] = make_promise_future(e);
      copy_promises.push_back(std::move(p));
      s.inputs[fwd.terms[i].node_id][fwd.terms[i].port] = std::move(f);
    }

    if(fwd.move_end == fwd.terms.size()) {
      std::optional<Promise> move_promise;
      if(fwd.copy_end != fwd.move_end) {
        const Iterm t = fwd.terms[fwd.copy_end];
        std::tie(move_promise, s.inputs[t.node_id][t.port]) = make_promise_future(e);
      }

      // move_only_function pls :(
      auto lambda_state = std::make_shared<std::pair<std::vector<Promise>, std::optional<Promise>>>(
        std::move(copy_promises), std::move(move_promise));
      std::move(outputs[i]).then([s = std::move(lambda_state)](Any a) mutable {
        for(Promise& p : s->first) std::move(p).send(a);
        if(s->second) {
          std::move(*s->second).send(std::move(a));
        }
      });
    } else {
      auto [borrowed_promise, f2] = make_promise_future(e);
      auto [borrowed_future, move_future] = borrow(std::move(f2));

      // move_only_function pls :(
      auto lambda_state = std::make_shared<std::pair<std::vector<Promise>, Promise>>(std::move(copy_promises),
                                                                                     std::move(borrowed_promise));
      std::move(outputs[i]).then([s = std::move(lambda_state)](Any a) mutable {
        for(Promise& p : s->first) std::move(p).send(a);
        std::move(s->second).send(std::move(a));
      });

      for(int i = fwd.move_end; i < fwd.terms.size(); i++) {
        s.borrowed_inputs[fwd.terms[i].node_id][fwd.terms[i].port] = borrowed_future;
      }

      if(fwd.copy_end != fwd.move_end) {
        const Iterm t = fwd.terms[fwd.copy_end];
        s.inputs[t.node_id][t.port] = std::move(move_future);
      }
    }
  }

  return s;
}

InputState
propagate(InputState s, Executor e, const std::vector<ValueForward>& fwds, std::vector<BorrowedFuture> outputs) {
  assert(fwds.size() == outputs.size());
  for(int i = 0; i < int(fwds.size()); i++) {

    for(int u = 0; u < fwds[i].copy_end; u++) {
      const Iterm t = fwds[i].terms[u];
      auto [p, f] = make_promise_future(e);
      s.inputs[t.node_id][t.port] = std::move(f);
      outputs[i].then([p = std::make_shared<Promise>(std::move(p))](const Any& a) { std::move(*p).send(a); });
    }

    for(int u = fwds[i].copy_end; u < int(fwds[i].terms.size()); u++) {
      const Iterm t = fwds[i].terms[u];
      s.borrowed_inputs[t.node_id][t.port] = outputs[i];
    }
  }
  return s;
}

} // namespace

std::vector<Future> execute_graph(const FunctionGraph& g,
                                  Executor executor,
                                  std::vector<Future> inputs,
                                  std::vector<BorrowedFuture> borrowed_inputs) {
  InputState s;
  s.inputs.reserve(g.owned_fwds.size());
  s.borrowed_inputs.reserve(g.functions.size());

  for(const auto& [owned, borrowed] : g.input_counts) {
    s.inputs.push_back(std::vector<Future>(owned));
    s.borrowed_inputs.push_back(std::vector<BorrowedFuture>(borrowed));
  }

  s.inputs.push_back(std::vector<Future>(g.output_types.size()));
  // can't return borrowed_inputs

  s = propagate(std::move(s), executor, g.owned_fwds.front(), std::move(inputs));
  s = propagate(std::move(s), executor, g.input_borrowed_fwds, std::move(borrowed_inputs));

  for(int i = 0; i < int(g.functions.size()); i++) {
    std::vector<Future> results =
      invoke_async(executor, g.functions[i], std::move(s.inputs[i]), std::move(s.borrowed_inputs[i]));
    s = propagate(std::move(s), executor, g.owned_fwds[i + 1], std::move(results));
  }

  return std::move(s.inputs.back());
}

std::vector<Any> execute_graph(const FunctionGraph& g, Executor executor, std::vector<Any> inputs) {
  std::vector<Future> input_futures;
  std::vector<BorrowedFuture> borrowed_input_futures;

  for(size_t i = 0; i < g.input_types.size(); i++) {
    Future f(executor, std::move(inputs[i]));
    if(g.input_types[i].value) {
      input_futures.push_back(std::move(f));
    } else {
      borrowed_input_futures.push_back(borrow(std::move(f)).first);
    }
  }

  std::vector<Future> result_futures =
    execute_graph(g, std::move(executor), std::move(input_futures), std::move(borrowed_input_futures));

  std::vector<Any> results;
  results.reserve(result_futures.size());
  for(Future& f : result_futures) {
    results.push_back(std::move(f).wait());
  }

  return results;
}

} // namespace anyf
