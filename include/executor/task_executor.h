#ifndef EXECUTOR_TASK_EXECUTOR_H
#define EXECUTOR_TASK_EXECUTOR_H

#include <cassert>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>

namespace anyf {

class task_queue {
  std::deque<std::tuple<std::function<void()>, int>> _q;
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

  std::optional<std::tuple<std::function<void()>, int>> pop() {
    std::unique_lock<std::mutex> lock{_mutex};
    while(_q.empty() && !_done)
      _cv.wait(lock);
    if(_q.empty())
      return std::nullopt;
    auto x = std::move(_q.front());
    _q.pop_front();
    return x;
  }

  std::optional<std::tuple<std::function<void()>, int>> try_pop() {
    std::unique_lock<std::mutex> lock{_mutex, std::try_to_lock};
    if(!lock || _q.empty())
      return std::nullopt;
    auto x = std::move(_q.front());
    _q.pop_front();
    return x;
  }

  template <typename F>
  void push(int task_group, F&& f) {
    {
      std::unique_lock<std::mutex> lock{_mutex};
      _q.emplace_back(std::forward<F>(f), task_group);
    }
    _cv.notify_one();
  }

  template <typename F>
  bool try_push(int task_group, F&& f) {
    {
      std::unique_lock<std::mutex> lock{_mutex, std::try_to_lock};
      if(!lock)
        return false;
      _q.emplace_back(std::forward<F>(f), task_group);
    }
    _cv.notify_one();

    return true;
  }
};

class task_executor {
  unsigned _count = std::thread::hardware_concurrency();
  int _next_task_group = 1;
  std::vector<std::thread> _threads;
  std::vector<task_queue> _q;
  std::atomic<unsigned> _index = 0;

  std::unordered_map<int, std::atomic<unsigned>> _group_task_counts;

  void run(unsigned i) {
    while(true) {
      unsigned spin_count = std::max<unsigned>(64, _count);
      for(unsigned n = 0; n < spin_count; n++) {
        auto tuple = _q[(i + n) % _count].try_pop();
        if(tuple) {
          std::get<0> (*tuple)();
          _group_task_counts[std::get<1>(*tuple)].fetch_sub(
              1, std::memory_order_relaxed);
          continue;
        }
      }

      auto tuple = _q[i].pop();
      if(!tuple)
        break;
      std::get<0> (*tuple)();
      _group_task_counts[std::get<1>(*tuple)].fetch_sub(
          1, std::memory_order_relaxed);
    }
  }

  void run_while_waiting(int task_group) {
    while(!is_task_group_complete(task_group)) {
      unsigned spin_count = std::max<unsigned>(64, _count);
      for(unsigned n = 0; n < spin_count; n++) {
        auto tuple = _q[(_count - 1 + n) % _count].try_pop();
        if(tuple) {
          std::get<0> (*tuple)();
          _group_task_counts[std::get<1>(*tuple)].fetch_sub(
              1, std::memory_order_relaxed);
          continue;
        }
      }
    }
  }

public:
  task_executor() : _q(_count) {
    for(unsigned i = 0; i < _count - 1; i++) {
      _threads.emplace_back([this, i]() { run(i); });
    }
  }

  task_executor(unsigned count) : _count(count), _q(_count) {
    for(unsigned i = 0; i < _count - 1; i++) {
      _threads.emplace_back([this, i]() { run(i); });
    }
  }

  ~task_executor() {
    for(auto& q : _q)
      q.done();
    for(auto& thread : _threads)
      thread.join();
  }

  template <typename F>
  void async(int task_group, F&& f) {
    auto i = _index++;
    _group_task_counts[task_group].fetch_add(1, std::memory_order_relaxed);

    const int K = 5;
    for(unsigned n = 0; n < _count * K; n++) {
      if(_q[(i + n) % _count].try_push(task_group, std::forward<F>(f)))
        return;
    }
    _q[i % _count].push(task_group, std::forward<F>(f));
  }

  int create_task_group() { return _next_task_group++; }

  bool is_task_group_complete(int task_group) const {
    auto it = _group_task_counts.find(task_group);

    assert(it != _group_task_counts.end());
    int val = it->second.load(std::memory_order_relaxed);

    return val == 0;
  }

  void wait_for_task_group(int task_group) { run_while_waiting(task_group); }
};

} // namespace anyf

#endif