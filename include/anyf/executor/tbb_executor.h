#pragma once

#include "anyf/executor.h"

#include <tbb/global_control.h>
#include <tbb/info.h>
#include <tbb/task_group.h>

namespace anyf {

class TBBExecutor {
  tbb::global_control _control;
  tbb::task_group _group;

public:
  TBBExecutor() : TBBExecutor(unsigned(tbb::info::default_concurrency()) - 1) {}
  explicit TBBExecutor(unsigned num_threads)
      : _control(tbb::global_control::max_allowed_parallelism, num_threads + 1) {}

  ~TBBExecutor() noexcept {}

  template <typename F>
  void run(F&& f) {
    // tbb requires a const call operator, even if the function is called once
    _group.run([f = std::forward<F>(f)]() { const_cast<std::decay_t<F>&>(f)(); });
  }

  void wait() { _group.wait(); }
};

inline Executor make_tbb_executor(unsigned num_threads = 0) {
  return num_threads == 0 ? make_executor<TBBExecutor>() : make_executor<TBBExecutor>(num_threads);
}

} // namespace anyf
