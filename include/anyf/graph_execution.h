#pragma once

#include "anyf/graph.h"
#include "anyf/static_graph.h"
#include "anyf/util.h"

#include "anyf/future.h"

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

template <typename Executor>
struct ExecutionCtx {
  Executor* executor = nullptr;

  // Storage for the results of all the functions,
  // First vector is the input values
  std::vector<std::vector<Any>> results;

  // Where to forward the results
  std::vector<std::vector<ValueForward>> fwds;

  std::vector<std::unique_ptr<FunctionTask>> tasks;

  std::vector<Promise> outputs;
};

template <typename Executor>
void execute_task(std::shared_ptr<ExecutionCtx<Executor>>& ctx, int idx);

template <typename Executor>
void propagate_ref(std::shared_ptr<ExecutionCtx<Executor>>& ctx, Term dst, Any* value) {
  ctx->tasks[dst.node_id - 1]->input_ptrs[dst.port] = value;

  if(decrement(ctx->tasks[dst.node_id - 1]->ref_count)) {
    (*ctx->executor)([ctx, i = dst.node_id - 1]() mutable { execute_task(ctx, i); });
  }
}

template <typename Executor>
void propagate_value(std::shared_ptr<ExecutionCtx<Executor>>& ctx, Term dst, Any value) {
  if(dst.node_id - 1 == ctx->tasks.size()) {
    std::move(ctx->outputs[dst.port]).send(*ctx->executor, std::move(value));
  } else {
    ctx->tasks[dst.node_id - 1]->input_vals[dst.port] = std::move(value);
    propagate_ref(ctx, dst, &ctx->tasks[dst.node_id - 1]->input_vals[dst.port]);
  }
}

template <typename Executor>
void propogate_outputs(std::shared_ptr<ExecutionCtx<Executor>>& ctx, const ValueForward& fwd, Any& value) {
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

template <typename Executor>
void execute_task(std::shared_ptr<ExecutionCtx<Executor>>& ctx, int idx) {
  ctx->results[idx + 1] = ctx->tasks[idx]->function(ctx->tasks[idx]->input_ptrs);

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

template <typename Executor>
std::vector<Future> execute_graph(const FunctionGraph& g, Executor& executor, std::vector<Future> inputs) {
  check(inputs.size() == input_types(g).size(), "expected {} inputs given {}", input_types(g).size(), inputs.size());

  auto ctx = std::make_shared<ExecutionCtx<Executor>>(ExecutionCtx<Executor>{&executor});

  ctx->results.resize(g.size() - 1);
  ctx->fwds.resize(g.size() - 1);

  ctx->results.front().resize(inputs.size());
  ctx->fwds.front().resize(inputs.size());

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

      const auto ref_begin = std::stable_partition(fwd.terms.begin(), fwd.terms.end(),
        [&](Term t) { return !input_type(g, t).is_cref(); });

      fwd.move_end = static_cast<int>(std::distance(fwd.terms.begin(), ref_begin));
      fwd.copy_end = (type.is_move_constructible() && fwd.move_end != 0) ? fwd.move_end - 1 : fwd.move_end;

      if(ref_begin != fwd.terms.end()) {
        RefCleanup* cleanup = new RefCleanup{
          static_cast<int>(std::distance(ref_begin, fwd.terms.end())),
          src,
          (type.is_copy_constructible() || fwd.copy_end == fwd.move_end)
            ? std::nullopt
            : std::optional(fwd.terms[fwd.copy_end])};

        std::for_each(ref_begin, fwd.terms.end(), [&](Term dst) {
          ctx->tasks[dst.node_id - 1]->ref_cleanups.push_back(cleanup);
        });

        if(type.is_copy_constructible() && fwd.copy_end != fwd.move_end) {
          fwd.copy_end++;
        }
      }

      it = next_it;
    }
  }

  // Create a promise/future for each output
  const size_t num_outputs = output_types(g).size();

  std::vector<Future> results;
  results.reserve(num_outputs);
  ctx->outputs.reserve(num_outputs);

  for(size_t i = 0; i < num_outputs; i++) {
    auto [p, f] = make_promise_future();
    ctx->outputs.push_back(std::move(p));
    results.push_back(std::move(f));
  }

  // Determine 0 input tasks to launch immediately
  for(size_t i = 1; i < g.size() - 1; i++) {
    if(ctx->tasks[i - 1]->function.input_types().empty()) {
      (*ctx->executor)([ctx, i = i - 1]() mutable { execute_task(ctx, i); });
    }
  }

  for(int i = 0; i < inputs.size(); i++) {
    std::move(inputs[i]).then(*ctx->executor, [ctx, i](Any value) mutable {
      ctx->results[0][i] = std::move(value);
      propogate_outputs(ctx, ctx->fwds[0][i], ctx->results[0][i]);
    });
  }

  return results;
}

template <typename Executor>
std::vector<Any> execute_graph(const FunctionGraph& g, Executor&& executor, std::vector<Any> inputs) {
  std::vector<Future> input_futures;
  input_futures.reserve(inputs.size());
  for(Any& any : inputs) {
    input_futures.emplace_back(std::move(any));
  }

  std::vector<Future> result_futures = execute_graph(g, executor, std::move(input_futures));

  std::vector<Any> results;
  results.reserve(result_futures.size());
  for(Future& f : result_futures) {
    results.push_back(std::move(f).wait());
  }

  return results;
}

template <typename... Outputs, typename Executor, typename... Inputs>
auto execute_graph(const StaticFunctionGraph<TypeList<Outputs...>, TypeList<std::decay_t<Inputs>...>>& g, Executor&& executor,
                   Inputs&&... inputs) {
  return apply_range<sizeof...(Outputs)>(
    execute_graph(g, std::move(executor), make_vector<Any>(std::forward<Inputs>(inputs)...)),
      [](auto&&... anys) { return tuple_or_value(any_cast<Outputs>(std::move(anys))...); });
}

} // namespace anyf
