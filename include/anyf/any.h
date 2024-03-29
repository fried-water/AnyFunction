#pragma once

#include "anyf/type.h"
#include "anyf/util.h"

#include <exception>

namespace anyf {

struct BadCast : std::runtime_error {
  BadCast() : std::runtime_error("Bad any cast") {}
};

struct BadCopy : std::runtime_error {
  BadCopy() : std::runtime_error("Trying to copy move-only type") {}
};

// Any implementation that works with move-only types (and throws when trying to copy them)
// TODO: sbo optimization
class Any {
  struct Concept {
    virtual ~Concept() = default;
    virtual std::unique_ptr<Concept> clone() const = 0;
  };

  template <typename T>
  struct Concrete final : Concept {
    Concrete(T&& t) : val(std::move(t)) {}
    Concrete(const T& t) : val(t) {}

    T val;

    std::unique_ptr<Concept> clone() const override {
      if constexpr(std::is_copy_constructible_v<T>) {
        return std::make_unique<Concrete<T>>(val);
      } else {
        throw BadCopy();
      }
    };
  };

  std::unique_ptr<Concept> _concept;
  TypeID _type = {};

public:
  Any() = default;

  template <typename T, typename = std::enable_if_t<!std::is_same_v<std::decay_t<T>, Any>>>
  Any(T&& t)
      : _concept(std::make_unique<Concrete<std::decay_t<T>>>(std::forward<T>(t))), _type(type_id(decay(Type<T>{}))) {}

  Any(Any&&) = default;
  Any& operator=(Any&&) = default;

  Any(const Any& a) : _concept(a._concept->clone()), _type(a._type) {}

  Any& operator=(const Any& a) {
    _concept = a._concept->clone();
    _type = a._type;
    return *this;
  }

  bool has_value() const { return _concept != nullptr; }
  TypeID type() const { return _type; }

  template <typename T>
  friend T& any_cast(Any&);

  template <typename T>
  friend const T& any_cast(const Any&);

  template <typename T>
  friend T&& any_cast(Any&&);

  template <typename T>
  friend T* any_cast(Any*);

  template <typename T>
  friend const T* any_cast(const Any*);
};

template <typename T>
T& any_cast(Any& any) {
  if(type_id(Type<T>{}) == any.type()) {
    return static_cast<Any::Concrete<T>&>(*any._concept).val;
  } else {
    throw BadCast();
  }
}

template <typename T>
const T& any_cast(const Any& any) {
  if(type_id(Type<T>{}) == any.type()) {
    return static_cast<const Any::Concrete<T>&>(*any._concept).val;
  } else {
    throw BadCast();
  }
}

template <typename T>
T&& any_cast(Any&& any) {
  if(type_id(Type<T>{}) == any.type()) {
    return std::move(static_cast<Any::Concrete<T>&>(*any._concept).val);
  } else {
    throw BadCast();
  }
}

template <typename T>
T* any_cast(Any* any) {
  if(type_id(Type<T>{}) == any->type()) {
    return &static_cast<Any::Concrete<T>&>(*any->_concept).val;
  } else {
    return nullptr;
  }
}

template <typename T>
const T* any_cast(const Any* any) {
  if(type_id(Type<T>{}) == any->type()) {
    return &static_cast<const Any::Concrete<T>&>(*any->_concept).val;
  } else {
    return nullptr;
  }
}

template <typename T>
bool holds_alternative(const Any& any) {
  return type_id(Type<T>{}) == any.type();
}

} // namespace anyf
