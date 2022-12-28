#pragma once

#include "anyf/executor.h"

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace anyf {

class TaskQueue {
  std::deque<std::function<void()>> _q;
  bool _done = false;
  std::mutex _mutex;
  std::condition_variable _cv;

public:
  void done() {
    {
      std::unique_lock<std::mutex> lock{_mutex};
      _done = true;
    }
    _cv.notify_all();
  }

  std::optional<std::function<void()>> pop() {
    std::unique_lock<std::mutex> lock{_mutex};
    _cv.wait(lock, [&]() { return _done || !_q.empty(); });
    if(_q.empty()) return std::nullopt;
    auto x = std::move(_q.front());
    _q.pop_front();
    return x;
  }

  std::optional<std::function<void()>> try_pop() {
    std::unique_lock<std::mutex> lock{_mutex, std::try_to_lock};
    if(!lock || _q.empty()) return std::nullopt;
    auto x = std::move(_q.front());
    _q.pop_front();
    return x;
  }

  template <typename F>
  void push(F&& f) {
    {
      std::unique_lock<std::mutex> lock{_mutex};
      _q.emplace_back(std::forward<F>(f));
    }
    _cv.notify_one();
  }

  template <typename F>
  bool try_push(F&& f) {
    {
      std::unique_lock<std::mutex> lock{_mutex, std::try_to_lock};
      if(!lock) return false;
      _q.emplace_back(std::forward<F>(f));
    }
    _cv.notify_one();

    return true;
  }
};

class TaskExecutor {
  std::vector<std::thread> _threads;
  std::vector<TaskQueue> _q;
  std::atomic<unsigned> _index = 0;
  std::atomic<unsigned> _in_flight = 1;

  void run(unsigned i, unsigned count) {
    while(_in_flight != 0) {
      unsigned spin_count = std::max<unsigned>(64, count);
      for(unsigned n = 0; n < spin_count; n++) {
        auto opt_function = _q[(i + n) % count].try_pop();
        if(opt_function) {
          (*opt_function)();
          _in_flight--;
          continue;
        }
      }

      auto opt_function = _q[i].pop();
      if(!opt_function) break;
      (*opt_function)();
      _in_flight--;
    }
  }

public:
  TaskExecutor() : TaskExecutor(std::thread::hardware_concurrency()) {}

  explicit TaskExecutor(unsigned num_threads) : _q(num_threads) {
    assert(num_threads > 0);
    for(unsigned i = 0; i < num_threads; i++) {
      _threads.emplace_back([this, i, num_threads]() { run(i, num_threads); });
    }
  }

  // must be called before destroyed
  void wait() {
    _in_flight--;

    for(auto& q : _q) q.done();
    for(auto& thread : _threads) thread.join();
    _q.clear();
    _threads.clear();
  }

  ~TaskExecutor() { assert(_q.empty() && _threads.empty()); }

  template <typename F>
  void run(F&& f) {
    _in_flight++;
    auto i = _index++;

    const int K = 5;
    for(unsigned n = 0; n < _threads.size() * K; n++) {
      if(_q[(i + n) % _threads.size()].try_push(std::forward<F>(f))) return;
    }
    _q[i % _threads.size()].push(std::forward<F>(f));
  }
};

inline Executor make_task_executor(unsigned num_threads = 0) {
  return num_threads == 0 ? make_executor<TaskExecutor>() : make_executor<TaskExecutor>(num_threads);
}

} // namespace anyf
