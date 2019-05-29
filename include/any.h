#ifndef ANY_H
#define ANY_H

#include <memory>
#include <typeinfo>
#include <type_traits>

namespace anyf {

struct BadAnyCast : public std::exception {
  const char* what() const noexcept override { return "bad any_cast"; }
};

class Any {
public:
  Any() = default;

  Any(const Any& other) : _data(other._data ? other._data->clone() : nullptr) {}
  Any(Any&& other) = default;

  template<typename T, typename = std::enable_if_t<!std::is_same_v<std::decay_t<T>, Any>>>
  Any(T&& value) : _data(std::make_unique<TypedHolder<std::decay_t<T>>>(std::forward<T>(value))) { }

  template<typename T, typename... Args>
  explicit Any(std::in_place_type_t<T>, Args&&... args) : _data(std::make_unique<TypedHolder<std::decay_t<T>>>(std::forward<Args>(args)...)) { }

  Any& operator=(const Any& other) {
    _data = other._data ? other._data->clone() : nullptr;
    return *this;
  }
  Any& operator=(Any&& other) = default;

  template<typename T, typename = std::enable_if_t<!std::is_same_v<std::decay_t<T>, Any>>>
  Any& operator=(T&& value) {
    _data = std::make_unique<TypedHolder<std::decay_t<T>>>(std::forward<T>(value));
    return *this;
  }

  ~Any() = default;

  template<typename T, typename... Args>
  T& emplace(Args&&... args) {
    _data = std::make_unique<TypedHolder<std::decay_t<T>>>(std::forward<Args>(args)...);
    return static_cast<TypedHolder<std::decay_t<T>>*>(_data.get())->value;
  }
  void reset() noexcept { _data.reset(); }

  bool empty() const noexcept { return _data == nullptr; }
  const std::type_info& type() const noexcept { return _data ? _data->type() : typeid(void); }

private:
  struct Holder {
    virtual ~Holder() = default;
    virtual const std::type_info& type() const noexcept = 0;
    virtual std::unique_ptr<Holder> clone() const = 0;
  };

  template<typename T>
  struct TypedHolder : Holder {
    template<typename... Args>
    TypedHolder(Args&&... args) : value(std::forward<Args>(args)...) { }

    const std::type_info& type() const noexcept override { return typeid(T); }
    std::unique_ptr<Holder> clone() const override { 
      return std::make_unique<TypedHolder<T>>(value);
    }

    T value;
  };

  template <typename T>
  friend T any_cast(Any);

  std::unique_ptr<Holder> _data;
};

template <typename T>
T any_cast(Any any) {
  static_assert(!std::is_same_v<void, std::decay_t<T>>);

  if(typeid(std::decay_t<T>) != any.type()) {
    throw BadAnyCast{};
  }
  return std::move(static_cast<Any::TypedHolder<std::decay_t<T>>*>(any._data.get())->value);
}

template<typename T, typename... Args>
Any make_any(Args&&... args) { return Any{std::in_place_type_t<T>{}, std::forward<Args>(args)...}; }

}

#endif
