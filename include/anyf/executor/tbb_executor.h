#pragma once

#include "tbb/global_control.h"
#include "tbb/info.h"
#include "tbb/task_group.h"

#include <unordered_map>

namespace anyf {

class TBBExecutor {
  tbb::global_control _control;
  std::unordered_map<int, tbb::task_group> _groups;
  int _next_task_group = 0;

public:
  TBBExecutor() : _control(tbb::global_control::max_allowed_parallelism, (size_t)tbb::info::default_concurrency) {}
  TBBExecutor(unsigned num_threads) : _control(tbb::global_control::max_allowed_parallelism, num_threads) {}

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
