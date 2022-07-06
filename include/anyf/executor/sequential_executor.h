#pragma once

#include <utility>

namespace anyf {

struct SequentialExecutor {
  SequentialExecutor() = default;
  // To match other executor constructors that take num_threads
  explicit SequentialExecutor(unsigned) {}

  template <typename F>
  void operator()(F&& f) { std::forward<F>(f)(); }
};

} // namespace anyf
