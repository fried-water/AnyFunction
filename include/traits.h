#pragma once

#include <functional>
#include <tuple>
#include <type_traits>

namespace anyf {

template <typename T>
struct Ty {
  using type = T;
};

template <typename... Ts>
struct TL {};

template <typename... Ts>
constexpr size_t size(TL<Ts...>) {
  return sizeof...(Ts);
}

template <typename T, typename... Ts>
constexpr auto head(TL<T, Ts...>) {
  return Ty<T>{};
}

template <typename T, typename... Ts>
constexpr auto tail(TL<T, Ts...>) {
  return TL<Ts...>{};
}

template <typename... Ts, typename F>
constexpr auto map(TL<Ts...>, F f) {
  return TL{typename decltype(f(Ty<Ts>{}))::type{}...};
}

template <typename... Ts, typename F>
constexpr auto map_value(TL<Ts...>, F f) {
  return std::array{f(Ty<Ts>{})...};
}

template <typename... Ts, typename F>
constexpr int count_if(TL<Ts...>, F f) {
  return ((f(Ty<Ts>{}) ? 1 : 0) + ...);
}

template <typename... Ts, typename F>
constexpr bool all(TL<Ts...>, F f) {
  return (f(Ty<Ts>{}) && ...);
}

template <typename... Ts, typename F>
constexpr bool any(TL<Ts...>, F f) {
  return (f(Ty<Ts>{}) || ...);
}

template <typename... Ts, typename F>
constexpr bool none(TL<Ts...>, F f) {
  return (!f(Ty<Ts>{}) && ...);
}

template <typename T, typename T2>
constexpr bool is_same(Ty<T>, Ty<T2>) {
  return std::is_same_v<T, T2>;
}

template <typename T>
constexpr bool is_pointer(Ty<T>) {
  return std::is_pointer_v<T>;
}

template <typename T>
constexpr bool is_decayed(Ty<T>) {
  return std::is_same_v<T, std::decay_t<T>>;
}

template <typename T>
constexpr bool is_const_ref(Ty<T>) {
  return std::is_same_v<T, const std::decay_t<T>&>;
}

template <typename T>
constexpr bool is_tuple(Ty<T>) {
  return false;
}

template <typename... Ts>
constexpr bool is_tuple(Ty<std::tuple<Ts...>>) {
  return true;
}

template <typename T>
constexpr auto decay(Ty<T>) {
  return Ty<std::decay_t<T>>{};
}

template <typename... Ts>
constexpr auto decay(TL<Ts...>) {
  return TL<std::decay_t<Ts>...>{};
}

constexpr auto as_tl(Ty<void>) { return TL<>{}; }

template <typename T>
constexpr auto as_tl(Ty<T>) {
  return TL<T>{};
}

template <typename... Ts>
constexpr auto as_tl(Ty<std::tuple<Ts...>>) {
  return TL<Ts...>{};
}

namespace detail {

template <typename>
struct function_traits;

template <typename Function>
struct function_traits : public function_traits<decltype(&Function::operator())> {};

template <typename Class, typename Ret, typename... Args>
struct function_traits<Ret (Class::*)(Args...) const> {
  static constexpr bool is_const = true;
  static constexpr Ty<Ret> return_type = {};
  static constexpr TL<Args...> args = {};
};

template <typename Class, typename Ret, typename... Args>
struct function_traits<Ret (Class::*)(Args...)> {
  static constexpr bool is_const = true;
  static constexpr Ty<Ret> return_type = {};
  static constexpr TL<Args...> args = {};
};

template <typename Ret, typename... Args>
struct function_traits<Ret (*)(Args...)> {
  static constexpr bool is_const = true;
  static constexpr Ty<Ret> return_type = {};
  static constexpr TL<Args...> args = {};
};

} // namespace detail

template <typename F>
constexpr auto return_type() {
  return detail::function_traits<F>::return_type;
}

template <typename F>
constexpr auto args() {
  return detail::function_traits<F>::args;
}

template <typename F>
constexpr auto arity() {
  return size(args<F>());
}

template <typename F>
constexpr auto num_outputs() {
  return size(as_tl(return_type<F>()));
}

template <typename F>
constexpr bool is_const() {
  return detail::function_traits<F>::is_const;
}

} // namespace anyf
