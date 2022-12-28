#pragma once

#include <knot/type_traits.h>

#include <functional>

namespace anyf {

class Executor {

  struct Model {
    virtual ~Model() = default;
    virtual void run(std::function<void()>) = 0;
    virtual void wait() = 0;
  };

  template <typename E>
  struct Concrete final : Model {
    E e;
    template <typename... Args>
    Concrete(Args&&... args) : e(std::forward<Args>(args)...) {}

    void run(std::function<void()> f) override { e.run(std::move(f)); }
    void wait() override { e.wait(); }
  };

  std::shared_ptr<Model> _e;

public:
  template <typename E, typename... Args>
  Executor(knot::Type<E>, Args&&... args) : _e(std::make_shared<Concrete<E>>(std::forward<Args>(args)...)) {}

  template <typename E>
  Executor(E e) : _e(std::make_shared<Concrete<E>>(std::move(e))) {}

  void run(std::function<void()> f) { _e->run(std::move(f)); }
  void wait() { _e->wait(); }

  explicit operator bool() const { return _e != nullptr; }
};

template <typename T, typename... Args>
Executor make_executor(Args&&... args) {
  return Executor(knot::Type<T>{}, std::forward<Args>(args)...);
}

} // namespace anyf
