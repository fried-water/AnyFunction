#ifndef EXECUTOR_H
#define EXECUTOR_H

#include "graph.h"

#include <boost/range/algorithm.hpp>
#include <boost/range/combine.hpp>

#include <atomic>
#include <memory>
#include <optional>

namespace anyf {

class alignas(128) function_task {
  std::atomic<int> _ref_count;

public:
  std::optional<any_value_function> func;
  small_vec<std::any, 3> input_vals;
  small_vec<std::any*, 3> input_ptrs;
  small_vec<std::pair<function_task*, int>, 3> output_vals;
  small_vec<std::pair<function_task*, int>, 3> output_refs;
  std::any result;

  int decrement_ref_count() {
    return _ref_count.fetch_sub(1, std::memory_order_release) - 1;
  }

  void set_ref_count(int x) { _ref_count.store(x, std::memory_order_release); }

  int ref_count() { return _ref_count.load(std::memory_order_acquire); }

  explicit function_task(std::optional<any_value_function> func, int num_inputs)
      : _ref_count(num_inputs), func(std::move(func)), input_vals(num_inputs),
        input_ptrs(num_inputs) {}
};

inline void propogate_outputs(function_task& task,
                              small_vec<function_task*, 3>& tasks_to_run) {
  assert(task.result.has_value() || task.func->output_type() == typeid(void));

  auto propogate_ref = [](const auto& pair, std::any& result,
                          small_vec<function_task*, 3>& tasks_to_run) {
    function_task* child_task = pair.first;
    child_task->input_ptrs[pair.second] = &result;

    if(child_task->decrement_ref_count() == 0) {
      tasks_to_run.push_back(child_task);
    }
  };

  auto propogate_val = [](const auto& pair, std::any result,
                          small_vec<function_task*, 3>& tasks_to_run) {
    function_task* child_task = pair.first;
    child_task->input_vals[pair.second] = std::move(result);
    child_task->input_ptrs[pair.second] = &child_task->input_vals[pair.second];

    if(child_task->decrement_ref_count() == 0) {
      tasks_to_run.push_back(child_task);
    }
  };

  std::for_each(task.output_refs.begin(), task.output_refs.end(),
                [&](const auto& pair) {
                  propogate_ref(pair, task.result, tasks_to_run);
                });

  if(task.output_refs.size() != 0) {
    // Copy all indices
    std::for_each(task.output_vals.begin(), task.output_vals.end(),
                  [&](const auto& pair) {
                    propogate_val(pair, task.result, tasks_to_run);
                  });
  } else if(task.output_vals.size() != 0) {
    // Copy indices 1..n-1
    std::for_each(task.output_vals.begin() + 1, task.output_vals.end(),
                  [&](const auto& pair) {
                    propogate_val(pair, task.result, tasks_to_run);
                  });

    // Move index 0
    propogate_val(task.output_vals.front(), std::move(task.result),
                  tasks_to_run);
  }
}

template <typename Executor, typename TaskGroupId>
void execute_task(Executor& executor, function_task& task, TaskGroupId id) {
  task.result = task.func->invoke(std::move(task.input_ptrs));

  small_vec<function_task*, 3> tasks_to_run;

  propogate_outputs(task, tasks_to_run);

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

  auto input_vec = util::make_vector<any_value_function::vec_val_type>(
      std::forward<Inputs>(inputs)...);
  int parameter_index = 0;

  // Create tasks
  boost::transform(
      g.nodes(), std::back_inserter(tasks), [&](const auto& variant) {
        return std::visit(
            [&](const auto& arg) {
              using T = std::decay_t<decltype(arg)>;
              if constexpr(std::is_same_v<T, Source>) {
                auto task = std::make_unique<function_task>(std::nullopt, 1);
                task->result = std::move(input_vec[parameter_index]);
                parameter_index++;

                return task;
              } else {
                return std::make_unique<function_task>(
                    std::get<InternalNode>(variant).func, arg.inputs.size());
              }
            },
            variant);
      });

  assert(tasks.size() == g.nodes().size());

  // Connect outputs
  for(int i = 0; i < static_cast<int>(g.nodes().size()); ++i) {
    for(const auto& output_pair : g.outputs(i)) {
      if(tasks[output_pair.first]->func->input_crefs()[output_pair.second]) {
        tasks[i]->output_refs.emplace_back(tasks[output_pair.first].get(),
                                           output_pair.second);
      } else {
        tasks[i]->output_vals.emplace_back(tasks[output_pair.first].get(),
                                           output_pair.second);
      }
    }
  }

  // Create task to hold result and connect it
  function_task result_task(std::nullopt, 2);
  tasks[g.graph_output()]->output_vals.emplace_back(&result_task, 0);

  small_vec<function_task*, 3> tasks_to_run;

  // Launch 0 input tasks
  boost::for_each(tasks, [&tasks_to_run](auto& task) {
    if(task->ref_count() == 0) {
      tasks_to_run.push_back(task.get());
    }
  });

  // Propogate inputs
  boost::for_each(boost::combine(g.nodes(), tasks),
                  [&tasks_to_run](auto&& tuple) {
                    const auto& variant = boost::get<0>(tuple);
                    function_task& task = *boost::get<1>(tuple);

                    if(std::holds_alternative<Source>(variant)) {
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

  return std::any_cast<Output>(std::move(result_task.input_vals[0]));
}

} // namespace anyf

#endif