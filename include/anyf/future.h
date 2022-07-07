#pragma once

#include "anyf/any.h"

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <variant>

namespace anyf {

inline bool decrement(std::atomic<int>& a) { return a.fetch_sub(1, std::memory_order_acq_rel) == 1; }
inline bool increment(std::atomic<int>& a) { return a.fetch_add(1, std::memory_order_acq_rel) == 0; }
inline bool ready(const std::atomic<int>& a) { return a.load(std::memory_order_acquire) == 0; }

struct NullExecutor {
  template<typename F> void operator()(F&&) const {}
};

struct SharedBlock {
  Any value;
  std::variant<
    std::monostate,
    std::function<void(Any)>,
    std::unique_ptr<std::pair<std::condition_variable, std::mutex>>> continuation;
  std::atomic<int> value_ready;
  std::atomic<int> ref_count;

  SharedBlock() : value_ready(1), ref_count(2) {}
  SharedBlock(Any&& value) : value(std::move(value)), value_ready(0), ref_count(1) {}
};

class Future;

class Promise {
 public:
  Promise() = default;
  ~Promise() { if(_block) std::move(*this).send(NullExecutor{}, Any{}); }

  Promise(const Promise&) = delete;
  Promise& operator=(const Promise&) = delete;

  Promise(Promise&& p) { *this = std::move(p); }
  Promise& operator=(Promise&& p) {
    _block = std::exchange(p._block, nullptr);
    return *this;
  }

  template<typename Executor>
  void send(Executor&& e, Any&& value) && {
    _block->value = std::move(value);

    if(decrement(_block->value_ready)) {
      if(_block->continuation.index() == 1) {
        e([f = std::move(std::get<1>(_block->continuation)), value = std::move(_block->value)]() mutable {
          f(std::move(value));
        });
      } else if(_block->continuation.index() == 2) {
        auto& [cv, mutex] = *std::get<2>(_block->continuation);
        std::scoped_lock lk(mutex);
        cv.notify_one();
      }
    }

    if(decrement(_block->ref_count)) delete _block;
    _block = nullptr;
  }

 private:
  SharedBlock* _block = nullptr;

  friend std::pair<Promise, Future> make_promise_future();
  Promise(SharedBlock* block) : _block(block) {}
};

class Future {
 public:
  Future() = default;
  explicit Future(Any&& value) : _block(new SharedBlock(std::move(value))) {}

  ~Future() { if(_block) std::move(*this).then(NullExecutor{}, [](Any){}); }

  Future(const Future&) = delete;
  Future& operator=(const Future&) = delete;

  Future(Future&& f) { *this = std::move(f); }
  Future& operator=(Future&& f) {
    _block = std::exchange(f._block, nullptr);
    return *this;
  }

  bool ready() const { return _block->value_ready.load(std::memory_order_relaxed) == 0; }

  Any wait() && {
    Any result;

    if(increment(_block->value_ready)) {
      result = std::move(_block->value);
    } else {
      _block->continuation = std::make_unique<std::pair<std::condition_variable, std::mutex>>();

      if(decrement(_block->value_ready)) {
        result = std::move(_block->value);
      } else {
        auto& [cv, mutex] = *std::get<2>(_block->continuation);
        std::unique_lock lk(mutex);
        cv.wait(lk, [&]() { return anyf::ready(_block->value_ready); });
        result = std::move(_block->value);
      }      
    }

    if(decrement(_block->ref_count)) delete _block;
    _block = nullptr;
    return result;
  }

  template<typename F, typename Executor>
  void then(Executor&& e, F&& f) && {
    if(increment(_block->value_ready)) {
      e([f = std::forward<F>(f), value = std::move(_block->value)]() mutable {
        f(std::move(value));
      });
    } else {
      _block->continuation = std::forward<F>(f);

      if(decrement(_block->value_ready)) {
        e([f = std::move(std::get<1>(_block->continuation)), value = std::move(_block->value)]() mutable {
          f(std::move(value));
        });
      }
    }

    if(decrement(_block->ref_count)) delete _block;
    _block = nullptr;
  }

 private:
  SharedBlock* _block;

  friend std::pair<Promise, Future> make_promise_future();
  Future(SharedBlock* block) : _block(std::move(block)) {}
};

inline std::pair<Promise, Future> make_promise_future() {
  auto block = new SharedBlock();
  return {Promise(block), Future(block)};
}

} // anyf
