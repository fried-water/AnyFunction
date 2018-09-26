#ifndef TASKS_H
#define TASKS_H

#include <boost/optional.hpp>

#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>

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

  boost::optional<std::function<void()>> pop() {
    std::unique_lock<std::mutex> lock{_mutex};
    while (_q.empty() && !_done) _cv.wait(lock);
    if(_q.empty()) return boost::none;
    auto x = std::move(_q.front());
    _q.pop_front();
    return x;
  }

  boost::optional<std::function<void()>> try_pop() {
    std::unique_lock<std::mutex> lock{_mutex, std::try_to_lock};
    if(!lock || _q.empty()) return boost::none;
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

class TaskSystem {
  const unsigned _count = std::thread::hardware_concurrency();
  std::vector<std::thread> _threads;
  std::vector<TaskQueue> _q;
  std::atomic<unsigned> _index = 0;

  void run(unsigned i) {
    while(true) {
      unsigned spin_count = std::max<unsigned>(16, _count);
      for(unsigned n = 0; n < spin_count; n++) {
        auto f = _q[(i+n) % _count].try_pop();
        if(f) {
          (*f)();
          continue;
        }
      }

      auto f = _q[i].pop();
      if(!f) break;
      (*f)();
    }
  }

 public:
  TaskSystem() : _q(_count) {
    for(unsigned i = 0; i < _count; i++) {
      _threads.emplace_back([this, i]() { run(i); });
    }
  }

  ~TaskSystem() {
    for(auto& q : _q) q.done();
    for(auto& thread : _threads) thread.join();
  }

  template <typename F>
  void async(int task_group, F&& f) {
    auto i = _index++;

    for(unsigned n = 0; n < _count; n++) {
       if(_q[(i+n) % _count].try_push(std::forward<F>(f))) return;
    }

    _q[i % _count].push(std::forward<F>(f));
  }

  // TODO
  int create_task_group() {
    return 0;
  }
  void wait_for_task_group(int task_group) { }

};

}


#endif