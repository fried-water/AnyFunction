#ifndef EXECUTOR_H
#define EXECUTOR_H

#include "graph.h"

#include <boost/range/algorithm.hpp>
#include <boost/range/combine.hpp>

#include <atomic>
#include <optional>
#include <memory>

namespace anyf {

class alignas(128) function_task {
  std::atomic<int> _ref_count;
 public:
  std::optional<any_function> func;
  small_vec<std::any, 3> inputs;
  std::vector<std::pair<function_task*, int>> outputs;
  std::any result;

  int decrement_ref_count() {
    return _ref_count.fetch_sub(1, std::memory_order_release) - 1;
  }

  void set_ref_count(int x) {
    _ref_count.store(x, std::memory_order_release);
  }

  int ref_count() {
    return _ref_count.load(std::memory_order_acquire);
  }

  explicit function_task(std::optional<any_function> func, int num_inputs)
    : _ref_count(num_inputs), 
      func(std::move(func)), 
      inputs(num_inputs) {}
};

inline void propogate_outputs(function_task& task, small_vec<function_task*, 3>& tasks_to_run) {
  assert(task.result.has_value());

  if(task.outputs.size() == 0) return;

  auto propogate = [](const auto& pair, std::any result, small_vec<function_task*, 3>& tasks_to_run) {
    function_task* child_task = pair.first;
    child_task->inputs[pair.second] = std::move(result);

    if(child_task->decrement_ref_count() == 0) {
      tasks_to_run.push_back(child_task);
    }
  };

  // Copy indices 1..n-1
  std::for_each(task.outputs.begin() + 1, task.outputs.end(), [&](const auto& pair) {
    propogate(pair, task.result, tasks_to_run);
  });

  // Move index 0
  propogate(task.outputs.front(), std::move(task.result), tasks_to_run);
}


template <typename Executor, typename TaskGroupId>
inline void execute_task(Executor& executor, function_task& task, TaskGroupId id) {
  
  task.result = task.func->invoke_any(std::move(task.inputs));

  small_vec<function_task*, 3> tasks_to_run;

  propogate_outputs(task, tasks_to_run);

  boost::for_each(tasks_to_run, [&executor, id](function_task* task) {
    executor.async(id, [&executor, task, id](){execute_task(executor, *task, id);});
  });
}

template <typename Output, typename Executor, typename... Inputs>
Output execute_graph(const function_graph<Output, Inputs...>& g, Executor& executor, Inputs&&... inputs) {
  std::vector<std::unique_ptr<function_task>> tasks;
  tasks.reserve(g.nodes().size());

  auto input_vec = util::make_vector<any_function::vec_type>(std::forward<Inputs>(inputs)...);
  int parameter_index = 0;

  // Create tasks
  boost::transform(g.nodes(), std::back_inserter(tasks), [&](const auto& variant) {
    return std::visit([&](const auto& arg) {
      using T = std::decay_t<decltype(arg)>;
      if constexpr (std::is_same_v<T, Source>) {
        auto task = std::make_unique<function_task>(std::nullopt, 1);
        task->result = std::move(input_vec[parameter_index]);
        parameter_index++;

        return task;
      } else if constexpr (std::is_same_v<T, Sink>) {
        return std::make_unique<function_task>(std::nullopt, 2);
      } else {
        return std::make_unique<function_task>(std::get<InternalNode>(variant).func, arg.inputs.size());
      }
    }, variant);
  });

  assert(tasks.size() == g.nodes().size());

  // Connect outputs
  for(int i = 0; i < static_cast<int>(g.nodes().size()); ++i) {
    for(const auto& output_pair : g.outputs(i)) {
      tasks[i]->outputs.emplace_back(tasks[output_pair.first].get(), output_pair.second);
    }
  }

  small_vec<function_task*, 3> tasks_to_run;

  // Launch 0 input tasks
  boost::for_each(tasks, [&tasks_to_run](auto& task) {
    if(task->ref_count() == 0) {
      tasks_to_run.push_back(task.get());
    }
  });

  // Propogate inputs
  boost::for_each(boost::combine(g.nodes(), tasks), [&tasks_to_run](auto&& tuple) {
    const auto& variant = boost::get<0>(tuple);
    function_task& task = *boost::get<1>(tuple);

    if(std::holds_alternative<Source>(variant)) {
      propogate_outputs(task, tasks_to_run);
    }
  });


  auto task_group_id = executor.create_task_group();

  // launch tasks
  boost::for_each(tasks_to_run, [&executor, task_group_id](function_task* task) {
    executor.async(task_group_id, [&executor, task, task_group_id](){execute_task(executor, *task, task_group_id);});
  });

  executor.wait_for_task_group(task_group_id);

  for(auto&& tuple : boost::combine(g.nodes(), tasks)) {
    const auto& variant = boost::get<0>(tuple);
    function_task& task = *boost::get<1>(tuple);

    if(std::holds_alternative<Sink>(variant)) {
      assert(task.ref_count() == 1);
      return std::any_cast<Output>(std::move(task.inputs[0]));
    }
  }

  throw 0;
}

}


#endif