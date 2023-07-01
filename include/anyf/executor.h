#pragma once

#include <knot/type_traits.h>

#include <atomic>
#include <exception>
#include <functional>

namespace anyf {

struct ExecutorModel {
  virtual ~ExecutorModel() noexcept(false) = default;
  virtual void run(std::function<void()>) = 0;
};

class ExecutorRef {
  ExecutorModel* _m;
  std::atomic<int>* _rc;

public:
  ExecutorRef(ExecutorModel* m, std::atomic<int>* rc) : _m(m), _rc(rc) { _rc->fetch_add(1, std::memory_order_relaxed); }

  ExecutorRef(const ExecutorRef& other) : _m(other._m), _rc(other._rc) { _rc->fetch_add(1, std::memory_order_relaxed); }

  ExecutorRef& operator=(const ExecutorRef& other) {
    _rc->fetch_sub(1, std::memory_order_relaxed);
    _m = other._m;
    _rc = other._rc;
    _rc->fetch_add(1, std::memory_order_relaxed);
    return *this;
  }

  ~ExecutorRef() { _rc->fetch_sub(1, std::memory_order_relaxed); }

  void run(std::function<void()> f) { _m->run(std::move(f)); }
};

class Executor {
  template <typename E>
  struct Concrete final : ExecutorModel {
    E e;
    template <typename... Args>
    Concrete(Args&&... args) : e(std::forward<Args>(args)...) {}
    void run(std::function<void()> f) override { e.run(std::move(f)); }
  };

  std::unique_ptr<ExecutorModel> _e;
  std::atomic<int> _rc = 0;

public:
  template <typename E, typename... Args>
  Executor(knot::Type<E>, Args&&... args) : _e(std::make_unique<Concrete<E>>(std::forward<Args>(args)...)) {}

  template <typename E>
  Executor(E e) : _e(std::unique_ptr<Concrete<E>>(std::move(e))) {}

  ~Executor() noexcept(false) {
    // Ensure all concurrent tasks complete and any refs they held are freed
    _e = nullptr;
    if(_rc.load(std::memory_order_relaxed) != 0) {
      throw std::logic_error("undestructed executor refs\n");
    }
  }

  Executor(const Executor&) = delete;
  Executor& operator=(const Executor&) = delete;

  Executor(Executor&&) = delete;
  Executor& operator=(Executor&&) = delete;

  ExecutorRef ref() { return {_e.get(), &_rc}; }
  operator ExecutorRef() { return ref(); }

  int ref_count() const { return _rc.load(std::memory_order_relaxed); }
};

template <typename T, typename... Args>
Executor make_executor(Args&&... args) {
  return Executor(knot::Type<T>{}, std::forward<Args>(args)...);
}

} // namespace anyf
