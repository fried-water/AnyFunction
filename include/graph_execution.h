#pragma once

#include "static_graph.h"
#include "util.h"

#include <algorithm>
#include <atomic>
#include <memory>
#include <optional>

namespace anyf {

class alignas(64) FunctionTask;

struct ParameterEdge {
  FunctionTask* task;
  int output_term;
  int input_term;
};

struct RefCleanup {
  std::atomic<int> ref_count;
  std::any* src;
  std::optional<ParameterEdge> dst;
};

class alignas(64) FunctionTask {
  std::atomic<int> _ref_count;

public:
  std::optional<graph::Execution> func;

  // Stores the values of the inputs for copy/move parameters
  std::vector<std::any> input_vals;

  // Points to the location of the inputs, input_vals for copies/moves
  // input_task.result of other tasks for refs
  std::vector<std::any*> input_ptrs;

  // Buffer to hold the results of the function
  std::vector<std::any> results;

  // Information on where to send the output
  std::vector<ParameterEdge> output_moves;
  std::vector<ParameterEdge> output_copies;
  std::vector<ParameterEdge> output_refs;

  // Clean up any references to data we used
  std::vector<RefCleanup*> ref_forwards;

  int decrement_ref_count() { return _ref_count.fetch_sub(1, std::memory_order_release) - 1; }
  int ref_count() { return _ref_count.load(std::memory_order_acquire); }

  explicit FunctionTask(std::optional<graph::Execution> func, int num_inputs)
      : _ref_count(num_inputs), func(std::move(func)), input_vals(num_inputs),
        input_ptrs(num_inputs), ref_forwards(num_inputs) {}
};

template <typename Anys, typename ToPtr>
void propogate_outputs(FunctionTask& task, std::vector<FunctionTask*>& tasks_to_run, Anys& results,
                       ToPtr to_ptr) {
  assert(std::all_of(results.begin(), results.end(),
                     [to_ptr](auto& any) { return to_ptr(any)->has_value(); }));

  auto propogate_ref = [](const ParameterEdge& edge, std::any* result,
                          std::vector<FunctionTask*>& tasks_to_run) {
    edge.task->input_ptrs[edge.input_term] = result;

    if(edge.task->decrement_ref_count() == 0) {
      tasks_to_run.push_back(edge.task);
    }
  };

  auto propogate_val = [](const ParameterEdge& edge, std::any result,
                          std::vector<FunctionTask*>& tasks_to_run) {
    edge.task->input_vals[edge.input_term] = std::move(result);
    edge.task->input_ptrs[edge.input_term] = &edge.task->input_vals[edge.input_term];

    if(edge.task->decrement_ref_count() == 0) {
      tasks_to_run.push_back(edge.task);
    }
  };

  // Copy all indices
  for(const auto& edge : task.output_copies) {
    propogate_val(edge, *to_ptr(results[edge.output_term]), tasks_to_run);
  }

  // Set up refs
  for(const auto& edge : task.output_refs) {
    propogate_ref(edge, to_ptr(results[edge.output_term]), tasks_to_run);
  }

  auto output_term_compare = [](const auto& e1, const auto& e2) {
    return e1.output_term < e2.output_term;
  };

  // Ensure output refs and moves are sorted by output_term
  assert(std::is_sorted(task.output_refs.begin(), task.output_refs.end(), output_term_compare));
  assert(std::is_sorted(task.output_moves.begin(), task.output_moves.end(), output_term_compare));

  auto ref_it = task.output_refs.begin();
  auto move_it = task.output_moves.begin();

  for(int i = 0; i < static_cast<int>(results.size()); i++) {
    const bool has_move = move_it != task.output_moves.end() && move_it->output_term == i;
    const bool has_ref = ref_it != task.output_refs.end() && ref_it->output_term == i;

    // If the output is taken by ref, set up data to properly clean up/forward the value
    if(has_ref) {
      auto ref_end = std::find_if(ref_it, task.output_refs.end(),
                                  [i](const auto& edge) { return edge.output_term != i; });

      // Give reference tasks information to clean up the value
      auto ref_cleanup_data =
          new RefCleanup{std::distance(ref_it, ref_end), to_ptr(results[i]),
                         has_move ? std::make_optional(*move_it) : std::nullopt};

      // Add ref cleanup
      std::for_each(ref_it, ref_end, [ref_cleanup_data](const auto& loc) {
        loc.task->ref_forwards[loc.input_term] = ref_cleanup_data;
      });

      ref_it = ref_end;

      // If output is not taken by ref and is supposed to be moved do it now
    } else if(has_move) {
      propogate_val(*move_it, std::move(*to_ptr(results[i])), tasks_to_run);
    }

    if(has_move)
      move_it++;
  }
}

template <typename Executor>
void execute_task(Executor& executor, FunctionTask& task, int task_group_id) {
  task.results = (*task.func)(executor, std::move(task.input_ptrs));

  std::vector<FunctionTask*> tasks_to_run;

  propogate_outputs(task, tasks_to_run, task.results, [](std::any& any) { return &any; });

  // If we took anything by reference we potentially have to clean up the value
  // or move it to its final destination
  for(RefCleanup* cleanup : task.ref_forwards) {
    if(cleanup && cleanup->ref_count.fetch_sub(1, std::memory_order_release) - 1 == 0) {
      if(cleanup->dst) {
        ParameterEdge dst = *cleanup->dst;

        dst.task->input_vals[dst.input_term] = std::move(*cleanup->src);
        dst.task->input_ptrs[dst.input_term] = &dst.task->input_vals[dst.input_term];

        if(dst.task->decrement_ref_count() == 0) {
          tasks_to_run.push_back(dst.task);
        }
      } else {
        cleanup->src->reset();
      }

      delete cleanup;
    }
  }

  for(FunctionTask* task : tasks_to_run) {
    executor.async(task_group_id, [&executor, task, task_group_id]() {
      execute_task(executor, *task, task_group_id);
    });
  }
}

template <typename... Outputs, typename Executor, typename... Inputs>
std::vector<std::any> execute_graph(const FunctionGraph<TL<Outputs...>, TL<Inputs...>>& g,
                                    Executor& executor,
                                    std::array<std::any*, sizeof...(Inputs)> inputs) {
  using namespace anyf::graph;

  std::vector<std::unique_ptr<FunctionTask>> tasks;
  tasks.reserve(g.nodes().size());

  // Create input task
  tasks.push_back(std::make_unique<FunctionTask>(std::nullopt, 1));

  // Create function tasks
  std::transform(g.nodes().begin() + 1, g.nodes().end() - 1, std::back_inserter(tasks),
                 [&](const auto& node) {
                   assert(node.func);
                   return std::make_unique<FunctionTask>(*node.func, node.func->num_inputs());
                 });

  // Create output task, sizeof(Outputs) + 1 so it never runs
  tasks.push_back(std::make_unique<FunctionTask>(std::nullopt, sizeof...(Outputs) + 1));

  // Connect outputs
  for(size_t i = 0; i < tasks.size(); i++) {
    const auto& node = g.nodes()[i];
    auto& task = tasks[i];

    for(auto it = node.outputs.begin(); it != node.outputs.end(); ++it) {
      const NodeEdge& edge = *it;
      ParameterEdge parameter_edge{tasks[edge.dst.node_id].get(), edge.src_arg, edge.dst.arg_idx};
      if(edge.pb == PassBy::move) {
        task->output_moves.push_back(parameter_edge);
      } else if(edge.pb == PassBy::ref) {
        task->output_refs.push_back(parameter_edge);
      } else {
        // If this output is not taken as a ref and not moved, convert last copy to a move
        const bool allow_move =
            std::find_if(node.outputs.begin(), it,
                         [edge](const NodeEdge& edge2) {
                           return edge.src_arg == edge2.src_arg && edge2.pb != PassBy::copy;
                         }) == it &&
            std::find_if(it + 1, node.outputs.end(), [edge](const NodeEdge& edge2) {
              return edge.src_arg == edge2.src_arg;
            }) == node.outputs.end();

        if(allow_move) {

          task->output_moves.push_back(parameter_edge);
        } else {
          task->output_copies.push_back(parameter_edge);
        }
      }
    }
  }

  std::vector<FunctionTask*> tasks_to_run;

  // Add 0 input tasks
  for(auto& task : tasks) {
    if(task->ref_count() == 0) {
      tasks_to_run.push_back(task.get());
    }
  }

  // Propogate inputs
  propogate_outputs(*tasks.front(), tasks_to_run, inputs, [](std::any* any) { return any; });

  auto task_group_id = executor.create_task_group();

  // launch tasks
  for(FunctionTask* task : tasks_to_run) {
    executor.async(task_group_id, [&executor, task, task_group_id]() {
      execute_task(executor, *task, task_group_id);
    });
  }

  executor.wait_for_task_group(task_group_id);

  return std::move(tasks.back()->input_vals);
}

template <std::size_t... Is>
auto any_ptrs(std::vector<std::any>& any_vals, std::index_sequence<Is...>) {
  return std::array<std::any*, sizeof...(Is)>{&any_vals[Is]...};
}

template <typename... Outputs, typename Executor, typename... Inputs>
std::conditional_t<sizeof...(Outputs) == 1, std::tuple_element_t<0, std::tuple<Outputs...>>,
                   std::tuple<Outputs...>>
execute_graph(const FunctionGraph<TL<Outputs...>, TL<std::decay_t<Inputs>...>>& g,
              Executor& executor, Inputs&&... inputs) {
  // This vector needs to survive for the runtime of execute graph
  auto any_values = make_vector<std::any>(std::forward<Inputs>(inputs)...);

  return apply_range<sizeof...(Outputs)>(
      execute_graph(g, executor,
                    any_ptrs(any_values, std::make_index_sequence<sizeof...(Inputs)>())),
      [](auto&&... anys) { return tuple_or_value(std::any_cast<Outputs>(std::move(anys))...); });
}

} // namespace anyf
