#pragma once

#include <knot/core.h>

#include <functional>
#include <tuple>
#include <type_traits>

namespace anyf {

using knot::Type;
using knot::TypeCategory;
using knot::TypeList;

template <typename... Ts, typename F>
constexpr bool all(TypeList<Ts...>, F f) {
  return (f(Type<Ts>{}) && ...);
}

template <typename... Ts, typename F>
constexpr bool any(TypeList<Ts...>, F f) {
  return (f(Type<Ts>{}) || ...);
}

template <typename... Ts, typename F>
constexpr bool none(TypeList<Ts...>, F f) {
  return (!f(Type<Ts>{}) && ...);
}

template <typename... Ts>
constexpr auto decay(TypeList<Ts...> tl) {
  return knot::map(tl, [](auto t) { return decay(t); });
}

template <typename T>
constexpr bool is_const(Type<T>) {
  return std::is_const_v<std::remove_reference_t<T>>;
}

template <typename T>
constexpr bool is_lref(Type<T>) {
  return std::is_lvalue_reference_v<T>;
}

template <typename T>
constexpr bool is_rref(Type<T> t) {
  return std::is_rvalue_reference_v<T> && !is_const(t);
}

template <typename T>
constexpr bool is_const_ref(Type<T> t) {
  return is_const(t) && is_lref(t);
}

template <typename... Ts>
constexpr bool is_tuple(Type<std::tuple<Ts...>>) {
  return true;
}

template <typename T>
constexpr bool is_tuple(Type<T>) {
  return false;
}

namespace detail {

template <typename>
struct function_traits;

template <typename Function>
struct function_traits : public function_traits<decltype(&Function::operator())> {};

template <typename Class, typename Ret, typename... Args>
struct function_traits<Ret (Class::*)(Args...) const> {
  static constexpr bool is_const = true;
  static constexpr Type<Ret> return_type = {};
  static constexpr TypeList<Args...> args = {};
};

template <typename Class, typename Ret, typename... Args>
struct function_traits<Ret (Class::*)(Args...)> {
  static constexpr bool is_const = true;
  static constexpr Type<Ret> return_type = {};
  static constexpr TypeList<Args...> args = {};
};

template <typename Ret, typename... Args>
struct function_traits<Ret (*)(Args...)> {
  static constexpr bool is_const = true;
  static constexpr Type<Ret> return_type = {};
  static constexpr TypeList<Args...> args = {};
};

} // namespace detail

template <typename F>
constexpr auto return_type(Type<F>) {
  return detail::function_traits<F>::return_type;
}

template <typename F>
constexpr auto return_types(Type<F> f) {
  const auto r = return_type(f);
  if constexpr(Type<void>{} == r) {
    return knot::typelist();
  } else if constexpr(is_tuple(r)) {
    return knot::as_typelist(r);
  } else {
    return knot::typelist(r);
  }
}

template <typename F>
constexpr auto args(Type<F>) {
  return detail::function_traits<F>::args;
}

template <typename F>
constexpr bool is_const_function(Type<F>) {
  return detail::function_traits<F>::is_const;
}

} // namespace anyf
