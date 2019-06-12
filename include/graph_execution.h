#ifndef GRAPH_EXECUTION_H
#define GRAPH_EXECUTION_H

#include "graph.h"
#include "util.h"

#include <boost/range/algorithm/for_each.hpp>
#include <boost/range/combine.hpp>

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
  SmallVec<std::any, 3> input_vals;

  // Points to the location of the inputs, input_vals for copies/moves
  // input_task.result for refs
  SmallVec<std::any*, 3> input_ptrs;

  // Buffer to hold the results of the function
  SmallVec<std::any, 1> results;

  // Information on where to send the output
  SmallVec<ParameterEdge, 3> output_moves;
  SmallVec<ParameterEdge, 3> output_copies;
  SmallVec<ParameterEdge, 3> output_refs;

  // Clean up any references to data we used
  SmallVec<RefCleanup*, 3> ref_forwards;

  int decrement_ref_count() { return _ref_count.fetch_sub(1, std::memory_order_release) - 1; }
  int ref_count() { return _ref_count.load(std::memory_order_acquire); }

  explicit FunctionTask(std::optional<graph::Execution> func, int num_inputs)
      : _ref_count(num_inputs), func(std::move(func)), input_vals(num_inputs),
        input_ptrs(num_inputs), ref_forwards(num_inputs) {}
};

template <typename Anys, typename ToPtr>
void propogate_outputs(FunctionTask& task, SmallVec<FunctionTask*, 10>& tasks_to_run, Anys& results, ToPtr to_ptr) {
  assert(std::all_of(results.begin(), results.end(),
                     [to_ptr](auto& any) { return to_ptr(any)->has_value(); }));

  auto propogate_ref = [](const ParameterEdge& edge, std::any* result,
                          SmallVec<FunctionTask*, 10>& tasks_to_run) {
    edge.task->input_ptrs[edge.input_term] = result;

    if(edge.task->decrement_ref_count() == 0) {
      tasks_to_run.push_back(edge.task);
    }
  };

  auto propogate_val = [](const ParameterEdge& edge, std::any result,
                          SmallVec<FunctionTask*, 10>& tasks_to_run) {
    edge.task->input_vals[edge.input_term] = std::move(result);
    edge.task->input_ptrs[edge.input_term] = &edge.task->input_vals[edge.input_term];

    if(edge.task->decrement_ref_count() == 0) {
      tasks_to_run.push_back(edge.task);
    }
  };

  // Copy all indices
  boost::for_each(task.output_copies, [&](const auto& edge) {
    propogate_val(edge, *to_ptr(results[edge.output_term]), tasks_to_run);
  });

  // Set up refs
  boost::for_each(task.output_refs, [&](const auto& edge) {
    propogate_ref(edge, to_ptr(results[edge.output_term]), tasks_to_run);
  });

  auto output_term_compare = [](const auto& e1, const auto& e2) {
    return e1.output_term < e2.output_term;
  };

  // Ensure output refs and moves are sorted by output_term
  assert(std::is_sorted(task.output_refs.begin(), task.output_refs.end(), output_term_compare));
  assert(std::is_sorted(task.output_moves.begin(), task.output_moves.end(), output_term_compare));

  auto ref_it = task.output_refs.begin();
  auto move_it = task.output_moves.begin();

  for(int i = 0; i < static_cast<int>(results.size()); i++) {
    bool has_move = move_it != task.output_moves.end() && move_it->output_term == i;
    bool has_ref = ref_it != task.output_refs.end() && ref_it->output_term == i;

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

template <typename Executor, typename TaskGroupId>
void execute_task(Executor& executor, FunctionTask& task, TaskGroupId id) {
  task.results = (*task.func)(executor, std::move(task.input_ptrs));

  SmallVec<FunctionTask*, 10> tasks_to_run;

  propogate_outputs(task, tasks_to_run, task.results, [](std::any& any) { return &any; });

  // If we took anything by reference we potentially have to clean up the value
  // or move it to its final destination
  boost::for_each(task.ref_forwards, [&tasks_to_run](RefCleanup* cleanup) {
    if(!cleanup)
      return;
    if(cleanup->ref_count.fetch_sub(1, std::memory_order_release) - 1 == 0) {
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
  });

  boost::for_each(tasks_to_run, [&executor, id](FunctionTask* task) {
    executor.async(id, [&executor, task, id]() { execute_task(executor, *task, id); });
  });
}

template <typename... Outputs, typename Executor, typename... Inputs>
SmallVec<std::any, 3>
execute_graph(const FunctionGraph<std::tuple<Outputs...>, std::tuple<Inputs...>>& g,
              Executor& executor, std::array<std::any*, sizeof...(Inputs)> inputs) {
  using namespace anyf::graph;

  SmallVec<std::unique_ptr<FunctionTask>, 10> tasks;
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
  boost::for_each(boost::combine(g.nodes(), tasks), [&tasks](const auto& tuple) {
    const auto& node = boost::get<0>(tuple);
    auto& task = boost::get<1>(tuple);

    for(auto it = node.outputs.begin(); it != node.outputs.end(); ++it) {
      const NodeEdge& edge = *it;
      ParameterEdge parameter_edge{tasks[edge.dst.node_id].get(), edge.src_arg, edge.dst.arg_idx};
      if (edge.pb == PassBy::move) {
        task->output_moves.push_back(parameter_edge);
      } else if(edge.pb == PassBy::ref) {
        task->output_refs.push_back(parameter_edge);
      } else {
        // If this output is not taken as a ref and not moved, convert last copy to a move
        bool allow_move = std::find_if(node.outputs.begin(), it, [edge](const NodeEdge& edge2) {
          return edge.src_arg == edge2.src_arg && edge2.pb != PassBy::copy;
        }) == it && 
          std::find_if(it+1, node.outputs.end(), [edge](const NodeEdge& edge2) {
          return edge.src_arg == edge2.src_arg;
        }) == node.outputs.end();

        if(allow_move) {
          task->output_moves.push_back(parameter_edge);
        } else {
          task->output_copies.push_back(parameter_edge);
        }
      } 
    }
  });

  SmallVec<FunctionTask*, 10> tasks_to_run;

  // Add 0 input tasks
  boost::for_each(tasks, [&tasks_to_run](auto& task) {
    if(task->ref_count() == 0) {
      tasks_to_run.push_back(task.get());
    }
  });

  // Propogate inputs
  propogate_outputs(*tasks.front(), tasks_to_run, inputs, [](std::any* any) { return any; });

  auto task_group_id = executor.create_task_group();

  // launch tasks
  boost::for_each(tasks_to_run, [&executor, task_group_id](FunctionTask* task) {
    executor.async(task_group_id, [&executor, task, task_group_id]() {
      execute_task(executor, *task, task_group_id);
    });
  });

  executor.wait_for_task_group(task_group_id);

  return std::move(tasks.back()->input_vals);
}

template <std::size_t... Is>
auto any_ptrs(SmallVec<std::any, 3>& any_vals, std::index_sequence<Is...>) {
  return std::array<std::any*, sizeof...(Is)>{&any_vals[Is]...};
}

template <typename... Outputs, typename Executor, typename... Inputs>
std::conditional_t<sizeof...(Outputs) == 1, std::tuple_element_t<0, std::tuple<Outputs...>>,
                   std::tuple<Outputs...>>
execute_graph(const FunctionGraph<std::tuple<Outputs...>, std::tuple<std::decay_t<Inputs>...>>& g,
              Executor& executor, Inputs&&... inputs) {
  // This vector needs to survive for the runtime of execute graph
  auto any_values = util::make_vector<SmallVec<std::any, 3>>(std::forward<Inputs>(inputs)...);

  auto results = execute_graph(g, executor, any_ptrs(any_values, std::make_index_sequence<sizeof...(Inputs)>()));

  if constexpr(sizeof...(Outputs) == 1) {
    return std::any_cast<Outputs...>(std::move(results[0]));
  } else {
    return util::vec_to_tuple<std::tuple<Outputs...>>(std::move(results));
  }
}

} // namespace anyf

#endif