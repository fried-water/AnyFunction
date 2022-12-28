#pragma once

#include "anyf/executor.h"

#include <utility>

namespace anyf {

struct SequentialExecutor {
  template <typename F>
  void run(F f) {
    f();
  }
  void wait() {}
};

inline Executor make_seq_executor() { return make_executor<SequentialExecutor>(); }

} // namespace anyf
