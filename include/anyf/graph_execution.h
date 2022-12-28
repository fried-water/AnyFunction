#pragma once

#include "anyf/borrowed_future.h"
#include "anyf/future.h"
#include "anyf/graph.h"
#include "anyf/static_graph.h"
#include "anyf/util.h"

#include <algorithm>
#include <atomic>
#include <memory>
#include <optional>

namespace anyf {

struct RefCleanup {
  std::atomic<int> ref_count;
  Term src;
  std::optional<Term> dst;
};

struct ValueForward {
  std::vector<Term> terms;
  int copy_end;
  int move_end;
};

struct FunctionTask {
  AnyFunction function;

  std::atomic<int> ref_count;

  // Store a reference to the borrowed inputs to ensure they are kept alive
  std::vector<BorrowedFuture> borrowed_inputs;

  // Stores the values of the inputs for copy/move parameters
  std::vector<Any> input_vals;

  // Points to the location of the inputs, input_vals for copies/moves
  // result of other tasks for refs
  std::vector<Any*> input_ptrs;

  // Determines which references this task is responsible
  // for cleaning up or forwarding when finished
  std::vector<RefCleanup*> ref_cleanups;

  FunctionTask(AnyFunction function_)
      : function(std::move(function_))
      , ref_count(static_cast<int>(function.input_types().size()))
      , input_vals(function.input_types().size())
      , input_ptrs(function.input_types().size()) {}
};

struct ExecutionCtx {
  Executor executor;

  // Storage for the results of all the functions,
  // First vector is the input values
  std::vector<std::vector<Any>> results;

  // Where to forward the results
  std::vector<std::vector<ValueForward>> fwds;

  std::vector<std::unique_ptr<FunctionTask>> tasks;

  std::vector<Promise> outputs;
};

void execute_task(std::shared_ptr<ExecutionCtx>& ctx, int idx);

inline void propagate_ref(std::shared_ptr<ExecutionCtx>& ctx, Term dst, Any* value) {
  ctx->tasks[dst.node_id - 1]->input_ptrs[dst.port] = value;

  if(decrement(ctx->tasks[dst.node_id - 1]->ref_count)) {
    ctx->executor.run([ctx, i = dst.node_id - 1]() mutable { execute_task(ctx, i); });
  }
}

inline void propagate_value(std::shared_ptr<ExecutionCtx>& ctx, Term dst, Any value) {
  if(dst.node_id - 1 == ctx->tasks.size()) {
    std::move(ctx->outputs[dst.port]).send(std::move(value));
  } else {
    ctx->tasks[dst.node_id - 1]->input_vals[dst.port] = std::move(value);
    propagate_ref(ctx, dst, &ctx->tasks[dst.node_id - 1]->input_vals[dst.port]);
  }
}

inline void propogate_outputs(std::shared_ptr<ExecutionCtx>& ctx, const ValueForward& fwd, Any& value) {
  for(int i = 0; i < fwd.copy_end; i++) {
    propagate_value(ctx, fwd.terms[i], value);
  }

  for(int i = fwd.copy_end; i < fwd.move_end; i++) {
    propagate_value(ctx, fwd.terms[i], std::move(value));
  }

  for(int i = fwd.move_end; i < static_cast<int>(fwd.terms.size()); i++) {
    propagate_ref(ctx, fwd.terms[i], &value);
  }
}

inline void execute_task(std::shared_ptr<ExecutionCtx>& ctx, int idx) {
  ctx->results[idx + 1] = ctx->tasks[idx]->function(ctx->tasks[idx]->input_ptrs);

  // Drop reference to all borrowed inputs so they can be forwarded asap
  ctx->tasks[idx]->borrowed_inputs.clear();

  for(size_t i = 0; i < ctx->results[idx + 1].size(); i++) {
    propogate_outputs(ctx, ctx->fwds[idx + 1][i], ctx->results[idx + 1][i]);
  }

  // If we took anything by reference we potentially have to clean up the value
  // or move it to its final destination
  for(RefCleanup* cleanup : ctx->tasks[idx]->ref_cleanups) {
    if(decrement(cleanup->ref_count)) {
      Any& src = ctx->results[cleanup->src.node_id][cleanup->src.port];

      if(cleanup->dst) {
        propagate_value(ctx, *cleanup->dst, std::move(src));
      } else {
        src = {};
      }

      delete cleanup;
    }
  }
}

inline std::vector<Future> execute_graph(const FunctionGraph& g,
                                         Executor executor,
                                         std::vector<Future> inputs,
                                         std::vector<BorrowedFuture> borrowed_inputs) {
  check(inputs.size() + borrowed_inputs.size() == input_types(g).size(),
        "expected {} inputs given {} + {}",
        input_types(g).size(),
        inputs.size(),
        borrowed_inputs.size());

  const size_t num_inputs = input_types(g).size();
  const size_t num_outputs = output_types(g).size();

  auto ctx = std::make_shared<ExecutionCtx>(ExecutionCtx{std::move(executor)});

  ctx->results.resize(g.size() - 1);
  ctx->fwds.resize(g.size() - 1);

  ctx->results.front().resize(num_inputs);
  ctx->fwds.front().resize(num_inputs);

  for(int i = 1; i < static_cast<int>(g.size() - 1); i++) {
    const AnyFunction& func = std::get<AnyFunction>(g[i].func);
    ctx->tasks.push_back(std::make_unique<FunctionTask>(func));
    ctx->fwds[i].resize(func.output_types().size());
  }

  // Connect outputs
  for(int i = 0; i < static_cast<int>(g.size()) - 1; i++) {
    const std::vector<Edge>& outputs = g[i].outputs;

    auto it = outputs.begin();
    while(it != outputs.end()) {
      const Term src{i, it->src_port};
      const TypeProperties type = output_type(g, src);

      const auto next_it = std::find_if(it + 1, outputs.end(), [&](Edge edge) { return src.port != edge.src_port; });

      ValueForward& fwd = ctx->fwds[src.node_id][src.port];
      std::transform(it, next_it, std::back_inserter(fwd.terms), [](Edge e) { return e.dst; });

      const auto ref_begin =
        std::stable_partition(fwd.terms.begin(), fwd.terms.end(), [&](Term t) { return input_type(g, t).value; });

      fwd.move_end = static_cast<int>(std::distance(fwd.terms.begin(), ref_begin));
      fwd.copy_end = type.value ? std::max(0, fwd.move_end - 1) : fwd.move_end;

      if(ref_begin != fwd.terms.end()) {
        RefCleanup* cleanup = new RefCleanup{static_cast<int>(std::distance(ref_begin, fwd.terms.end())),
                                             src,
                                             (is_copyable(type.id) || fwd.copy_end == fwd.move_end)
                                               ? std::nullopt
                                               : std::optional(fwd.terms[fwd.copy_end])};

        std::for_each(
          ref_begin, fwd.terms.end(), [&](Term dst) { ctx->tasks[dst.node_id - 1]->ref_cleanups.push_back(cleanup); });

        if(is_copyable(type.id) && fwd.copy_end != fwd.move_end) {
          fwd.copy_end++;
        }
      }

      it = next_it;
    }
  }

  // Create a promise/future for each output
  std::vector<Future> results;

  results.reserve(num_outputs);
  ctx->outputs.reserve(num_outputs);

  for(size_t i = 0; i < num_outputs; i++) {
    auto [p, f] = make_promise_future(ctx->executor);
    ctx->outputs.push_back(std::move(p));
    results.push_back(std::move(f));
  }

  // Determine 0 input tasks to launch immediately
  for(size_t i = 1; i < g.size() - 1; i++) {
    if(ctx->tasks[i - 1]->function.input_types().empty()) {
      ctx->executor.run([ctx, i = i - 1]() mutable { execute_task(ctx, i); });
    }
  }

  int owned_offset = 0;
  int borrowed_offset = 0;
  for(size_t i = 0; i < num_inputs; i++) {
    if(input_types(g)[i].value) {
      std::move(inputs[owned_offset++]).then([ctx, i](Any value) mutable {
        ctx->results[0][i] = std::move(value);
        propogate_outputs(ctx, ctx->fwds[0][i], ctx->results[0][i]);
      });
    } else {
      const ValueForward& fwd = ctx->fwds[0][i];
      for(int i = fwd.move_end; i < static_cast<int>(fwd.terms.size()); i++) {
        ctx->tasks[fwd.terms[i].node_id - 1]->borrowed_inputs.push_back(borrowed_inputs[borrowed_offset]);
      }

      borrowed_inputs[borrowed_offset++].then(
        [ctx, i](const Any& value) mutable { propogate_outputs(ctx, ctx->fwds[0][i], const_cast<Any&>(value)); });
    }
  }

  return results;
}

inline std::vector<Any> execute_graph(const FunctionGraph& g, Executor executor, std::vector<Any> inputs) {
  std::vector<Future> input_futures;
  std::vector<BorrowedFuture> borrowed_input_futures;

  for(size_t i = 0; i < input_types(g).size(); i++) {
    Future f(executor, std::move(inputs[i]));
    if(input_types(g)[i].value) {
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

template <typename... Outputs, typename... Inputs>
auto execute_graph(const StaticFunctionGraph<TypeList<Outputs...>, TypeList<std::decay_t<Inputs>...>>& g,
                   Executor executor,
                   Inputs&&... inputs) {
  return apply_range<sizeof...(Outputs)>(
    execute_graph(g, std::move(executor), make_vector<Any>(std::forward<Inputs>(inputs)...)),
    [](auto&&... anys) { return tuple_or_value(any_cast<Outputs>(std::move(anys))...); });
}

} // namespace anyf
