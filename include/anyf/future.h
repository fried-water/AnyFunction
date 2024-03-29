#pragma once

#include "anyf/any.h"
#include "anyf/executor.h"

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <variant>

namespace anyf {

inline bool decrement(std::atomic<int>& a) { return a.fetch_sub(1, std::memory_order_acq_rel) == 1; }
inline bool ready(const std::atomic<int>& a) { return a.load(std::memory_order_acquire) == 0; }

struct SharedBlock {
  Any value;
  std::mutex mutex;
  std::condition_variable cv;
  std::optional<std::function<void(Any)>> continuation;
  ExecutorRef executor;
  std::atomic<int> value_ready;

  SharedBlock(Any value, ExecutorRef executor, int ref_count = 0)
      : value(std::move(value)), executor(executor), value_ready(ref_count) {}

  SharedBlock(ExecutorRef executor) : SharedBlock(Any{}, executor, 1) {}
};

class Future;
class Promise;

std::pair<Promise, Future> make_promise_future(ExecutorRef);

class Promise {
public:
  Promise() = default;
  ~Promise() {
    if(_block) std::move(*this).send(Any{});
  }

  Promise(const Promise&) = delete;
  Promise& operator=(const Promise&) = delete;

  Promise(Promise&& p) { *this = std::move(p); }
  Promise& operator=(Promise&& p) {
    this->~Promise();
    _block = std::move(p._block);
    return *this;
  }

  void send(Any value) && {
    _block->value = std::move(value);

    std::unique_lock lk(_block->mutex);

    if(decrement(_block->value_ready)) {
      lk.unlock();
      _block->cv.notify_one();

      if(_block->continuation) {
        _block->executor.run(
          [f = std::move(*_block->continuation), value = std::move(_block->value)]() mutable { f(std::move(value)); });
      }
    }

    _block = nullptr;
  }

  explicit operator bool() const { return _block != nullptr; }
  bool valid() const { return static_cast<bool>(*this); }

private:
  std::shared_ptr<SharedBlock> _block;

  friend std::pair<Promise, Future> make_promise_future(ExecutorRef);

  Promise(std::shared_ptr<SharedBlock> block) : _block(std::move(block)) {}
};

class Future {
public:
  Future() = default;

  explicit Future(ExecutorRef executor, Any value)
      : _block(std::make_shared<SharedBlock>(std::move(value), std::move(executor))) {}

  Future(const Future&) = delete;
  Future& operator=(const Future&) = delete;

  Future(Future&& f) { *this = std::move(f); }
  Future& operator=(Future&& f) {
    _block = std::move(f._block);
    return *this;
  }

  bool ready() const { return _block->value_ready.load(std::memory_order_relaxed) == 0; }
  auto executor() const { return _block->executor; }

  Any wait() && {
    std::unique_lock lk(_block->mutex);
    _block->cv.wait(lk, [&]() { return ready(); });
    Any result = std::move(_block->value);
    _block = nullptr;
    return result;
  }

  template <typename F, typename = std::enable_if_t<std::is_same_v<void, std::invoke_result_t<F, Any>>>>
  void then(F f) && {
    if(_block->value_ready.fetch_add(1, std::memory_order_acquire) == 0) {
      _block->executor.run(
        [f = std::move(f), value = std::move(_block->value)]() mutable { std::move(f)(std::move(value)); });
    } else {
      _block->continuation = std::move(f);

      if(decrement(_block->value_ready)) {
        _block->executor.run([f = std::move(*_block->continuation), value = std::move(_block->value)]() mutable {
          std::move(f)(std::move(value));
        });
      }
    }

    _block = nullptr;
  }

  template <typename F, typename = std::enable_if_t<!std::is_same_v<void, std::invoke_result_t<F, Any>>>>
  Future then(F f) && {
    auto [p, new_future] = make_promise_future(_block->executor);

    // std::function requires copyable callables...
    auto shared_p = std::make_shared<Promise>(std::move(p));

    if(_block->value_ready.fetch_add(1, std::memory_order_acquire) == 0) {
      _block->executor.run([f = std::move(f), value = std::move(_block->value), p = std::move(shared_p)]() mutable {
        std::move(*p).send(std::move(f)(std::move(value)));
      });
    } else {
      _block->continuation = [f = std::move(f), p = std::move(shared_p)](Any value) mutable {
        std::move(*p).send(std::move(f)(std::move(value)));
      };

      if(decrement(_block->value_ready)) {
        _block->executor.run([f = std::move(*_block->continuation), value = std::move(_block->value)]() mutable {
          std::move(f)(std::move(value));
        });
      }
    }

    _block = nullptr;
    return std::move(new_future);
  }

  explicit operator bool() const { return _block != nullptr; }
  bool valid() const { return static_cast<bool>(*this); }

private:
  std::shared_ptr<SharedBlock> _block;

  friend std::pair<Promise, Future> make_promise_future(ExecutorRef);

  Future(std::shared_ptr<SharedBlock> block) : _block(std::move(block)) {}
};

inline std::pair<Promise, Future> make_promise_future(ExecutorRef executor) {
  auto block = std::make_shared<SharedBlock>(std::move(executor));
  return {Promise(block), Future(block)};
}

} // namespace anyf
