#pragma once

#include "anyf/future.h"

namespace anyf {

struct BorrowedSharedBlock {
  Promise promise;
  Any value;
  std::mutex mutex;
  std::condition_variable cv;
  std::vector<std::function<void(const Any&)>> continuations;
  std::function<void(std::function<void()>)> executor;
  bool value_ready = false;

  BorrowedSharedBlock(Promise&& p, std::function<void(std::function<void()>)> executor)
      : promise(std::move(p)), executor(std::move(executor)) {}

  template <typename Executor>
  BorrowedSharedBlock(Promise&& p, Executor& executor)
      : BorrowedSharedBlock(std::move(p),
                            [e_ptr = &executor, this](std::function<void()> f) { (*e_ptr)(std::move(f)); }) {}

  ~BorrowedSharedBlock() { std::move(promise).send(std::move(value)); }
};

class BorrowedFuture {
public:
  BorrowedFuture() = default;

  const Any& wait() {
    std::unique_lock lk(_block->mutex);
    _block->cv.wait(lk, [&]() { return _block->value_ready; });
    return _block->value;
  }

  template <typename F, typename = std::enable_if_t<std::is_same_v<void, std::invoke_result_t<F, const Any&>>>>
  void then(F&& f) {
    std::unique_lock lk(_block->mutex);

    if(_block->value_ready) {
      lk.unlock();
      _block->executor([f = std::forward<F>(f), b = _block]() mutable {
        f(b->value);
        b = nullptr;
      });
    } else {
      _block->continuations.emplace_back(std::forward<F>(f));
    }
  }

  template <typename F, typename = std::enable_if_t<!std::is_same_v<void, std::invoke_result_t<F, const Any&>>>>
  Future then(F&& f) {
    auto [p, new_future] = make_promise_future(_block->executor);
    auto shared_p = std::make_shared<Promise>(std::move(p));

    std::unique_lock lk(_block->mutex);

    if(_block->value_ready) {
      lk.unlock();
      _block->executor([f = std::forward<F>(f), b = _block, p = std::move(shared_p)]() mutable {
        std::move(*p).send(std::forward<F>(f)(b->value));
        b = nullptr;
      });
    } else {
      _block->continuations.emplace_back([f = std::forward<F>(f), p = std::move(shared_p)](const Any& value) mutable {
        std::move(*p).send(std::forward<F>(f)(value));
      });
    }

    return std::move(new_future);
  }

private:
  std::shared_ptr<BorrowedSharedBlock> _block;

  friend std::pair<BorrowedFuture, Future> borrow(Future);

  BorrowedFuture(std::shared_ptr<BorrowedSharedBlock> block) : _block(std::move(block)) {}
};

inline std::pair<BorrowedFuture, Future> borrow(Future f) {
  auto executor = f.executor();
  auto [new_p, new_f] = make_promise_future(executor);

  auto block = std::make_shared<BorrowedSharedBlock>(std::move(new_p), executor);

  std::move(f).then([b = block](Any value) mutable {
    b->value = std::move(value);

    {
      std::lock_guard lk(b->mutex);
      b->value_ready = true;
    }
    b->cv.notify_all();

    for(auto&& f : b->continuations) {
      b->executor([f = std::move(f), b]() mutable {
        f(b->value);
        b = nullptr;
      });
    }
    b->continuations.clear();
    b = nullptr;
  });

  return {BorrowedFuture(std::move(block)), std::move(new_f)};
}

} // namespace anyf
