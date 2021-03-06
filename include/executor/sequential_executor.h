#ifndef EXECUTOR_SEQUENTIAL_EXECUTOR_H
#define EXECUTOR_SEQUENTIAL_EXECUTOR_H

namespace anyf {

struct SequentialExecutor {
  SequentialExecutor() = default;
  // To match other executor constructors that take num_threads
  SequentialExecutor(int) {}

  template <typename F>
  void async(int, F&& f) {
    f();
  }

  int create_task_group() { return 0; }
  void wait_for_task_group(int) {}
};

} // namespace anyf

#endif