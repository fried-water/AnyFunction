#pragma once

#include "graph.h"
#include "static_graph.h"
#include "util.h"

#include <algorithm>
#include <atomic>
#include <memory>
#include <optional>

namespace anyf {

inline bool decrement(std::atomic<int>& a) {
  return a.fetch_sub(1, std::memory_order_acq_rel) == 1;
}
inline bool ready(const std::atomic<int>& a) { return a.load(std::memory_order_acquire) == 0; }

struct RefCleanup {
  std::atomic<int> ref_count;
  graph::Term src;
  std::optional<graph::Term> dst;
};

struct alignas(64) FunctionTask {
  std::atomic<int> ref_count;

  // Stores the values of the inputs for copy/move parameters
  std::vector<std::any> input_vals;

  // Points to the location of the inputs, input_vals for copies/moves
  // result of other tasks for refs
  std::vector<std::any*> input_ptrs;

  std::vector<std::any> results;

  // Information on where to send the outputs
  std::vector<graph::Edge> moves;
  std::vector<graph::Edge> copies;
  std::vector<graph::Edge> refs;

  // Determines which references each node is respondible for cleaning up
  // or perhaps forwarding when finished
  std::vector<RefCleanup*> ref_cleanups;

  explicit FunctionTask(int num_inputs)
      : ref_count(num_inputs), input_vals(num_inputs), input_ptrs(num_inputs) {}
};

inline void propogate_outputs(std::vector<std::unique_ptr<FunctionTask>>& tasks,
                              std::vector<int>& tasks_to_run, int idx) {
  FunctionTask& task = *tasks[idx];
  std::vector<std::any>& results = task.results;

  const auto setup_input = [&](graph::Term dst, std::any* result) {
    tasks[dst.node_id]->input_ptrs[dst.port] = result;

    if(decrement(tasks[dst.node_id]->ref_count)) {
      tasks_to_run.push_back(dst.node_id);
    }
  };

  for(const auto [src_port, dst] : task.copies) {
    tasks[dst.node_id]->input_vals[dst.port] = results[src_port];
    setup_input(dst, &tasks[dst.node_id]->input_vals[dst.port]);
  }

  for(const auto [src_port, dst] : task.moves) {
    tasks[dst.node_id]->input_vals[dst.port] = std::move(results[src_port]);
    setup_input(dst, &tasks[dst.node_id]->input_vals[dst.port]);
  }

  for(const auto [src_port, dst] : task.refs) {
    setup_input(dst, &results[src_port]);
  }
}

template <typename Executor>
void execute_task(const FunctionGraph& g, std::vector<std::unique_ptr<FunctionTask>>& tasks,
                  Executor& executor, int idx, int task_group_id) {
  FunctionTask& task = *tasks[idx];

  task.results = std::get<AnyFunction>(g[idx].func)(task.input_ptrs);

  std::vector<int> tasks_to_run;

  propogate_outputs(tasks, tasks_to_run, idx);

  // If we took anything by reference we potentially have to clean up the value
  // or move it to its final destination
  for(RefCleanup* cleanup : task.ref_cleanups) {
    if(decrement(cleanup->ref_count)) {
      const graph::Term src = cleanup->src;

      if(cleanup->dst) {
        const graph::Term dst = *cleanup->dst;
        FunctionTask& dst_task = *tasks[dst.node_id];

        dst_task.input_vals[dst.port] = std::move(tasks[src.node_id]->results[src.port]);
        dst_task.input_ptrs[dst.port] = &dst_task.input_vals[dst.port];

        if(decrement(dst_task.ref_count)) {
          tasks_to_run.push_back(dst.node_id);
        }
      } else {
        tasks[src.node_id]->results[src.port].reset();
      }

      delete cleanup;
    }
  }

  for(int task : tasks_to_run) {
    executor.async(task_group_id, [&, task, task_group_id]() {
      execute_task(g, tasks, executor, task, task_group_id);
    });
  }
}

template <typename Executor>
std::vector<std::any> execute_graph(const FunctionGraph& g, Executor&& executor,
                                    std::vector<std::any> inputs) {
  using namespace anyf::graph;

  std::vector<std::unique_ptr<FunctionTask>> tasks;

  for(const auto& node : g) {
    tasks.push_back(std::make_unique<FunctionTask>(
        std::visit(Overloaded{// +1 so output and input task never run;
                              [](const std::vector<Type>& v) { return v.size() + 1; },
                              [](const AnyFunction& a) { return a.input_types().size(); }},
                   node.func)));
  }

  tasks.front()->results = std::move(inputs);

  // Connect outputs
  for(int i = 0; i < static_cast<int>(g.size()) - 1; i++) {
    const std::vector<Edge>& outputs = g[i].outputs;

    auto it = outputs.begin();
    while(it != outputs.end()) {
      const graph::Term src{i, it->src_port};
      const Type output_type = graph::output_type(g, src);

      const auto next_it = std::find_if(
          it + 1, outputs.end(), [&](graph::Edge edge) { return src.port != edge.src_port; });

      const int num_refs = static_cast<int>(std::count_if(
          it, next_it, [&](graph::Edge e) { return graph::input_type(g, e.dst).is_ref(); }));
      std::optional<graph::Term> move_term;

      RefCleanup* cleanup = num_refs > 0 ? new RefCleanup{num_refs, src, std::nullopt} : nullptr;

      std::for_each(it, next_it, [&](graph::Edge edge) {
        if(graph::input_type(g, edge.dst).is_ref()) {
          tasks[edge.dst.node_id]->ref_cleanups.push_back(cleanup);
          tasks[i]->refs.push_back(edge);
        } else if(output_type.is_move_constructible() && !move_term) {
          move_term = edge.dst;
        } else {
          tasks[i]->copies.push_back(edge);
        }
      });

      if(num_refs > 0) {
        cleanup->dst = output_type.is_copy_constructible() ? std::nullopt : move_term;

        if(move_term && output_type.is_copy_constructible()) {
          tasks[i]->copies.push_back({src.port, *move_term});
        }
      } else if(move_term) {
        tasks[i]->moves.push_back({src.port, *move_term});
      }

      it = next_it;
    }
  }

  std::vector<int> tasks_to_run;

  // Add 0 input tasks
  for(int i = 1; i < g.size() - 1; i++) {
    if(ready(tasks[i]->ref_count)) {
      tasks_to_run.push_back(i);
    }
  }

  // Propogate inputs
  propogate_outputs(tasks, tasks_to_run, 0);

  const auto task_group_id = executor.create_task_group();

  // launch tasks
  for(int task : tasks_to_run) {
    executor.async(task_group_id, [&, task, task_group_id]() {
      execute_task(g, tasks, executor, task, task_group_id);
    });
  }

  executor.wait_for_task_group(task_group_id);

  check(tasks.back()->ref_count == 1, "Output ref count not 1: {}", tasks.back()->ref_count.load());

  return std::move(tasks.back()->input_vals);
}

template <typename... Outputs, typename Executor, typename... Inputs>
auto execute_graph(const StaticFunctionGraph<TL<Outputs...>, TL<std::decay_t<Inputs>...>>& g,
                   Executor&& executor, Inputs&&... inputs) {
  return apply_range<sizeof...(Outputs)>(
      execute_graph(g, std::forward<Executor>(executor),
                    make_vector<std::any>(std::forward<Inputs>(inputs)...)),
      [](auto&&... anys) { return tuple_or_value(std::any_cast<Outputs>(std::move(anys))...); });
}

} // namespace anyf
