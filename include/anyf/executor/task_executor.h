#pragma once

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
    if(_q.empty())
      return std::nullopt;
    auto x = std::move(_q.front());
    _q.pop_front();
    return x;
  }

  std::optional<std::function<void()>> try_pop() {
    std::unique_lock<std::mutex> lock{_mutex, std::try_to_lock};
    if(!lock || _q.empty())
      return std::nullopt;
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
      if(!lock)
        return false;
      _q.emplace_back(std::forward<F>(f));
    }
    _cv.notify_one();

    return true;
  }
};

class TaskExecutor {
  unsigned _count;
  std::vector<std::thread> _threads;
  std::vector<TaskQueue> _q;
  std::atomic<unsigned> _index = 0;
  std::atomic<unsigned> _in_flight = 1;

  void run(unsigned i) {
    while(_in_flight != 0) {
      unsigned spin_count = std::max<unsigned>(64, _count);
      for(unsigned n = 0; n < spin_count; n++) {
        auto opt_function = _q[(i + n) % _count].try_pop();
        if(opt_function) {
          (*opt_function)();
          _in_flight--;
          continue;
        }
      }

      auto opt_function = _q[i].pop();
      if(!opt_function)
        break;
      (*opt_function)();
      _in_flight--;
    }
  }

public:
  TaskExecutor() : TaskExecutor(std::thread::hardware_concurrency()) {}

  explicit TaskExecutor(unsigned count) : _count(count), _q(_count) {
    for(unsigned i = 0; i < _count; i++) {
      _threads.emplace_back([this, i]() { run(i); });
    }
  }

  ~TaskExecutor() {
    _in_flight--;

    for(auto& q : _q)
      q.done();

    for(auto& thread : _threads)
      thread.join();
  }

  template <typename F>
  void operator()(F&& f) {
    _in_flight++;
    auto i = _index++;

    const int K = 5;
    for(unsigned n = 0; n < _count * K; n++) {
      if(_q[(i + n) % _count].try_push(std::forward<F>(f)))
        return;
    }
    _q[i % _count].push(std::forward<F>(f));
  }
};

} // namespace anyf
