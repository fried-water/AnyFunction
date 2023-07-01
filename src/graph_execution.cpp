#include "anyf/graph_execution.h"

#include "graph_inner.h"

#include <algorithm>
#include <atomic>
#include <memory>
#include <optional>

namespace anyf {

namespace {

struct InvocationBlock {
  std::shared_ptr<const AnyFunction> f;

  std::atomic<int> ref_count;

  std::vector<Any> input_vals;
  std::vector<Any*> input_ptrs;
  std::vector<BorrowedFuture> borrowed_inputs;

  std::vector<Promise> promises;

  InvocationBlock(std::shared_ptr<const AnyFunction> f_,
                  std::vector<BorrowedFuture> borrowed_inputs,
                  std::vector<Promise> promises)
      : f(std::move(f_))
      , ref_count(f->input_types().size())
      , input_vals(f->input_types().size())
      , input_ptrs(f->input_types().size())
      , borrowed_inputs(std::move(borrowed_inputs))
      , promises(std::move(promises)) {}
};

struct IfBlock {
  ExecutorRef e;
  std::atomic<int> ref_count;
  IfExpr expr;
  std::vector<Future> owned_inputs;
  std::vector<BorrowedFuture> borrowed_inputs;
  std::vector<Promise> promises;
};

struct SelectBlock {
  std::atomic<int> ref_count;
  std::vector<Future> if_branch;
  std::vector<Future> else_branch;
  std::vector<Promise> promises;
};

struct WhileBlock {
  ExecutorRef e;
  std::atomic<int> ref_count;
  WhileExpr w;
  std::vector<Future> owned_inputs;
  std::vector<BorrowedFuture> borrowed_inputs;
  std::vector<Promise> promises;
};

struct InputState {
  std::vector<std::vector<Future>> inputs;
  std::vector<std::vector<BorrowedFuture>> borrowed_inputs;
};

void invoke_async(ExecutorRef e,
                  const std::shared_ptr<const AnyFunction>& f,
                  std::vector<Promise> promises,
                  std::vector<Future> inputs,
                  std::vector<BorrowedFuture> borrowed_inputs) {
  InvocationBlock* block = new InvocationBlock(f, std::move(borrowed_inputs), std::move(promises));

  const auto invoke = [](InvocationBlock* block) {
    auto results = (*block->f)(block->input_ptrs);

    // Drop reference to all borrowed inputs so they can be forwarded asap
    block->borrowed_inputs.clear();

    // Forward outputs
    for(size_t i = 0; i < results.size(); i++) {
      std::move(block->promises[i]).send(std::move(results[i]));
    }

    delete block;
  };

  if(f->input_types().empty()) {
    e.run([block, invoke]() { invoke(block); });
  }

  int owned_offset = 0;
  int borrowed_offset = 0;
  for(size_t i = 0; i < f->input_types().size(); i++) {
    if(f->input_types()[i].value) {
      std::move(inputs[owned_offset++]).then([i, block, invoke](Any value) mutable {
        block->input_vals[i] = std::move(value);
        block->input_ptrs[i] = &block->input_vals[i];
        if(decrement(block->ref_count) == 1) {
          invoke(block);
        }
      });
    } else {
      block->borrowed_inputs[borrowed_offset++].then([i, block, invoke](const Any& value) {
        block->input_ptrs[i] = const_cast<Any*>(&value);
        if(decrement(block->ref_count) == 1) {
          invoke(block);
        }
      });
    }
  }
}

void invoke_async(Future cond, IfBlock* b) {
  std::move(cond).then([b](Any cond) {
    std::vector<Future> results = execute_graph(any_cast<bool>(cond) ? b->expr.if_branch : b->expr.else_branch,
                                                b->e,
                                                std::move(b->owned_inputs),
                                                b->borrowed_inputs);

    for(int i = 0; i < int(results.size()); i++) {
      std::move(results[i]).then([i, b](Any a) mutable {
        std::move(b->promises[i]).send(std::move(a));
        if(decrement(b->ref_count) == 1) {
          delete b;
        }
      });
    }
  });
}

void invoke_async(Future cond, SelectBlock* b) {
  std::move(cond).then([b](Any cond) {
    std::vector<Future> results = any_cast<bool>(cond) ? std::move(b->if_branch) : std::move(b->else_branch);

    for(int i = 0; i < int(results.size()); i++) {
      std::move(results[i]).then([i, b](Any a) mutable {
        std::move(b->promises[i]).send(std::move(a));
        if(decrement(b->ref_count) == 1) {
          delete b;
        }
      });
    }
  });
}

void invoke_async(Future cond, WhileBlock* b) {
  std::move(cond).then([b](Any cond) {
    if(any_cast<bool>(cond)) {
      auto res = execute_graph(b->w.body, b->e, std::move(b->owned_inputs), b->borrowed_inputs);
      b->owned_inputs =
        std::vector<Future>(std::make_move_iterator(res.begin() + 1), std::make_move_iterator(res.end()));
      invoke_async(std::move(res.front()), b);
    } else {
      for(int i = 0; i < b->owned_inputs.size(); i++) {
        std::move(b->owned_inputs[i]).then([i, b](Any a) mutable {
          std::move(b->promises[i]).send(std::move(a));
          if(decrement(b->ref_count) == 1) {
            delete b;
          }
        });
      }

      if(decrement(b->ref_count) == 1) {
        delete b;
      }
    }
  });
}

InputState propagate(InputState s, ExecutorRef e, const std::vector<ValueForward>& fwds, std::vector<Future> outputs) {
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
      auto lambda_state =
        new std::pair<std::vector<Promise>, std::optional<Promise>>(std::move(copy_promises), std::move(move_promise));
      std::move(outputs[i]).then([=](Any a) mutable {
        for(Promise& p : lambda_state->first) std::move(p).send(a);
        if(lambda_state->second) {
          std::move(*lambda_state->second).send(std::move(a));
        }
        delete lambda_state;
      });
    } else {
      auto [borrowed_promise, f2] = make_promise_future(e);
      auto [borrowed_future, move_future] = borrow(std::move(f2));

      // move_only_function pls :(
      auto lambda_state =
        new std::pair<std::vector<Promise>, Promise>(std::move(copy_promises), std::move(borrowed_promise));
      std::move(outputs[i]).then([=](Any a) mutable {
        for(Promise& p : lambda_state->first) std::move(p).send(a);
        std::move(lambda_state->second).send(std::move(a));
        delete lambda_state;
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
propagate(InputState s, ExecutorRef e, const std::vector<ValueForward>& fwds, std::vector<BorrowedFuture> outputs) {
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

std::vector<Future> execute_graph(const FunctionGraph& g_outer,
                                  ExecutorRef executor,
                                  std::vector<Future> inputs,
                                  std::vector<BorrowedFuture> borrowed_inputs) {
  const FunctionGraph::State& g = *g_outer.state;

  InputState s;
  s.inputs.reserve(g.owned_fwds.size());
  s.borrowed_inputs.reserve(g.exprs.size());

  for(const auto& [owned, borrowed] : g.input_counts) {
    s.inputs.push_back(std::vector<Future>(owned));
    s.borrowed_inputs.push_back(std::vector<BorrowedFuture>(borrowed));
  }

  s.inputs.push_back(std::vector<Future>(g.output_types.size()));
  // can't return borrowed_inputs

  s = propagate(std::move(s), executor, g.owned_fwds.front(), std::move(inputs));
  s = propagate(std::move(s), executor, g.input_borrowed_fwds, std::move(borrowed_inputs));

  for(int i = 0; i < int(g.exprs.size()); i++) {
    const int count = num_outputs(g.exprs[i]);

    std::vector<Promise> promises;
    std::vector<Future> futures;

    promises.reserve(count);
    futures.reserve(count);

    for(int i = 0; i < count; i++) {
      auto [p, f] = make_promise_future(executor);
      promises.push_back(std::move(p));
      futures.push_back(std::move(f));
    }

    std::visit(
      Overloaded{
        [&](const auto& f) {
          invoke_async(executor, f, std::move(promises), std::move(s.inputs[i]), std::move(s.borrowed_inputs[i]));
        },
        [&](const IfExpr& e) {
          invoke_async(std::move(s.inputs[i].front()),
                       new IfBlock{executor,
                                   int(promises.size()),
                                   e,
                                   std::vector<Future>(std::make_move_iterator(s.inputs[i].begin() + 1),
                                                       std::make_move_iterator(s.inputs[i].end())),
                                   std::move(s.borrowed_inputs[i]),
                                   std::move(promises)});
        },
        [&](const SelectExpr& e) {
          invoke_async(
            std::move(s.inputs[i].front()),
            new SelectBlock{int(promises.size()),
                            std::vector<Future>(std::make_move_iterator(s.inputs[i].begin() + 1),
                                                std::make_move_iterator(s.inputs[i].begin() + 1 + promises.size())),
                            std::vector<Future>(std::make_move_iterator(s.inputs[i].begin() + 1 + promises.size()),
                                                std::make_move_iterator(s.inputs[i].end())),
                            std::move(promises)});
        },
        [&](const WhileExpr& e) {
          invoke_async(std::move(s.inputs[i].front()),
                       new WhileBlock{executor,
                                      int(promises.size()) + 1,
                                      e,
                                      std::vector<Future>(std::make_move_iterator(s.inputs[i].begin() + 1),
                                                          std::make_move_iterator(s.inputs[i].end())),
                                      std::move(s.borrowed_inputs[i]),
                                      std::move(promises)});
        }},
      g.exprs[i]);

    s = propagate(std::move(s), executor, g.owned_fwds[i + 1], std::move(futures));
  }

  return std::move(s.inputs.back());
}

std::vector<Any> execute_graph(const FunctionGraph& g, ExecutorRef executor, std::vector<Any> inputs) {
  std::vector<Future> input_futures;
  std::vector<BorrowedFuture> borrowed_input_futures;

  for(size_t i = 0; i < g.state->input_types.size(); i++) {
    Future f(executor, std::move(inputs[i]));
    if(g.state->input_types[i].value) {
      input_futures.push_back(std::move(f));
    } else {
      borrowed_input_futures.push_back(borrow(std::move(f)).first);
    }
  }

  std::vector<Future> result_futures =
    execute_graph(g, executor, std::move(input_futures), std::move(borrowed_input_futures));

  std::vector<Any> results;
  results.reserve(result_futures.size());
  for(Future& f : result_futures) {
    results.push_back(std::move(f).wait());
  }

  return results;
}

} // namespace anyf
