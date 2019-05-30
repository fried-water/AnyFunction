#ifndef EXECUTOR_TBB_EXECUTOR_H
#define EXECUTOR_TBB_EXECUTOR_H

#include "tbb/task_group.h"
#include "tbb/task_scheduler_init.h"

#include <unordered_map>

namespace anyf {

class tbb_executor {
  tbb::task_scheduler_init _scheduler;
  std::unordered_map<int, tbb::task_group> _groups;
  int _next_task_group = 0;

public:
  tbb_executor() = default;
  tbb_executor(unsigned num_threads) : _scheduler(num_threads) {}

  template <typename F>
  void async(int task_group, F&& f) {
    _groups[task_group].run(std::forward<F>(f));
  }

  int create_task_group() {
    _groups[_next_task_group];
    return _next_task_group++;
  }

  void wait_for_task_group(int task_group) {
    _groups[task_group].wait();
    _groups.erase(task_group);
  }
};

} // namespace anyf

#endif