#ifndef GRAPH_EXECUTION_H
#define GRAPH_EXECUTION_H

#include "graph.h"

#include <boost/range/algorithm/for_each.hpp>
#include <boost/range/combine.hpp>

#include <atomic>
#include <memory>
#include <optional>

namespace anyf {

class alignas(128) function_task;

struct parameter_location {
  explicit parameter_location(function_task* task, int index)
      : task(task), index(index) {}
  function_task* task;
  int index;
};

struct ref_cleanup {
  ref_cleanup(int count, std::any& src, std::optional<parameter_location> dst)
      : ref_count(count), src(src), dst(dst) {}

  std::atomic<int> ref_count;
  std::any& src;
  std::optional<parameter_location> dst;
};

class alignas(128) function_task {
  std::atomic<int> _ref_count;

public:
  std::optional<any_function> func;

  // Stores the values of the inputs for copy/move parameters
  small_vec3<std::any> input_vals;

  // Points to the location of the inputs, input_vals for copies/moves
  // input_task.result for refs
  small_vec3<std::any*> input_ptrs;

  // Buffer to hold the result of the function
  std::any result;

  // Information on where to send the output
  std::optional<parameter_location> output_move;
  small_vec3<parameter_location> output_copies;
  small_vec3<parameter_location> output_refs;

  // Clean up any references to data we used
  small_vec3<std::shared_ptr<ref_cleanup>> ref_forwards;

  int decrement_ref_count() {
    return _ref_count.fetch_sub(1, std::memory_order_release) - 1;
  }

  int ref_count() { return _ref_count.load(std::memory_order_acquire); }

  explicit function_task(std::optional<any_function> func, int num_inputs)
      : _ref_count(num_inputs), func(std::move(func)), input_vals(num_inputs),
        input_ptrs(num_inputs) {}
};

inline void propogate_outputs(function_task& task,
                              small_vec3<function_task*>& tasks_to_run) {
  assert(task.result.has_value() || task.func->output_type() == typeid(void));

  auto propogate_ref = [](const parameter_location& loc, std::any& result,
                          small_vec3<function_task*>& tasks_to_run) {
    loc.task->input_ptrs[loc.index] = &result;

    if(loc.task->decrement_ref_count() == 0) {
      tasks_to_run.push_back(loc.task);
    }
  };

  auto propogate_val = [](const parameter_location& loc, std::any result,
                          small_vec3<function_task*>& tasks_to_run) {
    loc.task->input_vals[loc.index] = std::move(result);
    loc.task->input_ptrs[loc.index] = &loc.task->input_vals[loc.index];

    if(loc.task->decrement_ref_count() == 0) {
      tasks_to_run.push_back(loc.task);
    }
  };

  // Copy all indices
  boost::for_each(task.output_copies, [&](const auto& loc) {
    propogate_val(loc, task.result, tasks_to_run);
  });

  // Set up refs
  boost::for_each(task.output_refs, [&](const auto& loc) {
    propogate_ref(loc, task.result, tasks_to_run);
  });

  // If the output is taken by ref, set up data to properly clean up/forward the
  // value
  if(task.output_refs.size() != 0) {
    // Give reference tasks information to clean up the value
    auto ref_cleanup_data = std::make_shared<ref_cleanup>(
        task.output_refs.size(), task.result, task.output_move);

    // Add ref cleanup
    boost::for_each(task.output_refs, [&ref_cleanup_data](const auto& loc) {
      loc.task->ref_forwards.push_back(ref_cleanup_data);
    });
    // If output is not taken by ref and is supposed to be moved do it now
  } else if(task.output_move) {
    propogate_val(*task.output_move, std::move(task.result), tasks_to_run);
  }
}

template <typename Executor, typename TaskGroupId>
void execute_task(Executor& executor, function_task& task, TaskGroupId id) {
  task.result = task.func->invoke(std::move(task.input_ptrs));

  small_vec3<function_task*> tasks_to_run;

  propogate_outputs(task, tasks_to_run);

  // If we took anything by reference we potentially have to clean up the value
  // or move it to its final destination
  boost::for_each(
      task.ref_forwards,
      [&tasks_to_run](std::shared_ptr<ref_cleanup>& cleanup) {
        if(cleanup->ref_count.fetch_sub(1, std::memory_order_release) - 1 ==
           0) {
          if(cleanup->dst) {
            parameter_location dst = *cleanup->dst;

            dst.task->input_vals[dst.index] = std::move(cleanup->src);
            dst.task->input_ptrs[dst.index] = &dst.task->input_vals[dst.index];

            if(dst.task->decrement_ref_count() == 0) {
              tasks_to_run.push_back(dst.task);
            }

          } else {
            cleanup->src.reset();
          }
        }
      });

  boost::for_each(tasks_to_run, [&executor, id](function_task* task) {
    executor.async(
        id, [&executor, task, id]() { execute_task(executor, *task, id); });
  });
}

template <typename Output, typename Executor, typename... Inputs>
Output execute_graph(const function_graph<Output, std::decay_t<Inputs>...>& g,
                     Executor& executor, Inputs&&... inputs) {
  std::vector<std::unique_ptr<function_task>> tasks;
  tasks.reserve(g.nodes().size());

  auto input_vec = util::make_vector<any_function::vec_val_type>(
      std::forward<Inputs>(inputs)...);
  int parameter_index = 0;

  // Create tasks
  boost::transform(g.nodes(), std::back_inserter(tasks), [&](const auto& node) {
    return std::visit(
        [&](const auto& arg) {
          using T = std::decay_t<decltype(arg)>;
          if constexpr(std::is_same_v<T, graph::source>) {
            auto task = std::make_unique<function_task>(std::nullopt, 1);
            task->result = std::move(input_vec[parameter_index]);
            parameter_index++;

            return task;
          } else if constexpr(std::is_same_v<T, graph::func_node>) {
            return std::make_unique<function_task>(arg.func, arg.func.input_types().size());
          } else {
            return std::make_unique<function_task>(std::nullopt, 2);
          }
        },
        node.variant);
  });

  // Connect outputs
  for(int i = 0; i < static_cast<int>(g.nodes().size() - 1); ++i) {
    for(graph::edge edge : g.nodes()[i].outputs) {
      if(edge.pb == pass_by::ref) {
        tasks[i]->output_refs.emplace_back(tasks[edge.dst_id].get(),
                                           edge.arg_idx);
      } else if(edge.pb == pass_by::copy) {
        tasks[i]->output_copies.emplace_back(tasks[edge.dst_id].get(),
                                             edge.arg_idx);
      } else {
        tasks[i]->output_move.emplace(tasks[edge.dst_id].get(), edge.arg_idx);
      }
    }
  }

  small_vec3<function_task*> tasks_to_run;

  // Add 0 input tasks
  boost::for_each(tasks, [&tasks_to_run](auto& task) {
    if(task->ref_count() == 0) {
      tasks_to_run.push_back(task.get());
    }
  });

  // Propogate inputs
  boost::for_each(boost::combine(g.nodes(), tasks),
                  [&tasks_to_run](auto&& tuple) {
                    const auto& node = boost::get<0>(tuple);
                    function_task& task = *boost::get<1>(tuple);

                    if(std::holds_alternative<graph::source>(node.variant)) {
                      propogate_outputs(task, tasks_to_run);
                    }
                  });

  auto task_group_id = executor.create_task_group();

  // launch tasks
  boost::for_each(
      tasks_to_run, [&executor, task_group_id](function_task* task) {
        executor.async(task_group_id, [&executor, task, task_group_id]() {
          execute_task(executor, *task, task_group_id);
        });
      });

  executor.wait_for_task_group(task_group_id);

  return std::any_cast<Output>(std::move(tasks.back()->input_vals[0]));
}

} // namespace anyf

#endif