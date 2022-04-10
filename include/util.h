#pragma once

#include <any>
#include <functional>
#include <tuple>
#include <vector>

namespace anyf {

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
decltype(auto) apply_range(Range&& range, F f, std::index_sequence<Is...>) {
  if constexpr(std::is_reference_v<Range>) {
    return f(range[Is]...);
  } else {
    return f(std::move(range[Is])...);
  }
}

} // namespace details

template <size_t Count, typename Range, typename F>
decltype(auto) apply_range(Range&& range, F f) {
  return details::apply_range(std::forward<Range>(range), f, std::make_index_sequence<Count>());
}

} // namespace anyf
