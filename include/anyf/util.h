#pragma once

#include "anyf/traits.h"

#include <fmt/core.h>

#include <array>
#include <optional>
#include <tuple>
#include <utility>
#include <vector>

namespace anyf {

template <class... Ts>
struct Overloaded : Ts... {
  using Ts::operator()...;
};
template <class... Ts>
Overloaded(Ts...) -> Overloaded<Ts...>;

template <typename T, typename... Elements>
std::vector<T> make_vector(Elements&&... elements) {
  std::vector<T> vec;
  vec.reserve(sizeof...(Elements));
  (vec.emplace_back(std::forward<Elements>(elements)), ...);
  return vec;
}

template <typename... Ts>
decltype(auto) tuple_or_value(Ts&&... t) {
  if constexpr(sizeof...(Ts) == 1) {
    return std::forward<Ts...>(t...);
  } else {
    return std::tuple(std::forward<Ts>(t)...);
  }
}

namespace details {

template <typename Range, typename F, std::size_t... Is>
decltype(auto) apply_range(Range&& range, F&& f, std::index_sequence<Is...>) {
  if constexpr(std::is_reference_v<Range>) {
    return std::forward<F>(f)(range[Is]...);
  } else {
    return std::forward<F>(f)(std::move(range[Is])...);
  }
}

} // namespace details

template <size_t Count, typename Range, typename F>
decltype(auto) apply_range(Range&& range, F f) {
  return details::apply_range(std::forward<Range>(range), f, std::make_index_sequence<Count>());
}

template <typename F, typename... Inputs>
decltype(auto) invoke_normalize_void_return(F&& f, Inputs&&... inputs) {
  if constexpr(Type<void>{} == return_type(decay(Type<F>({})))) {
    std::forward<F>(f)(std::forward<Inputs>(inputs)...);
    return std::tuple();
  } else {
    return std::forward<F>(f)(std::forward<Inputs>(inputs)...);
  }
}

template <typename... Ts>
[[noreturn]] void error(const char* fmt_string, const Ts&... ts) {
  fmt::print("Error: {}\n", fmt::format(fmt_string, ts...));
  exit(1);
}

template <typename... Ts>
void check(bool b, const char* fmt_string, const Ts&... ts) {
  if(!b) {
    error(fmt_string, ts...);
  }
}

} // namespace anyf
