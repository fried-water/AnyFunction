#ifndef UTIL_H
#define UTIL_H

#include <boost/container/small_vector.hpp>

#include <any>
#include <functional>
#include <tuple>
#include <vector>

namespace anyf {

template <typename T, std::size_t N>
using small_vec = boost::container::small_vector<T, N>;

template <typename T>
using small_vec_base = boost::container::small_vector_base<T>;

namespace util {

template <typename Vec, typename... Elements>
Vec make_vector(Elements&&... elements) {
  Vec vec;
  vec.reserve(sizeof...(Elements));
  (vec.emplace_back(std::forward<Elements>(elements)), ...);
  return vec;
}

template <typename T, typename... Elements>
std::vector<T> make_std_vector(Elements&&... elements) {
  std::vector<T> vec;
  vec.reserve(sizeof...(Elements));
  (vec.emplace_back(std::forward<Elements>(elements)), ...);
  return vec;
}

template <typename T, typename... Elements>
small_vec<T, 3> make_small_vector(Elements&&... elements) {
  small_vec<T, 3> vec;
  vec.reserve(sizeof...(Elements));
  (vec.emplace_back(std::forward<Elements>(elements)), ...);
  return vec;
}

template <typename OutputContainer, typename InputContainer, typename Transform>
OutputContainer map(InputContainer&& input_container, Transform transform) {
  OutputContainer out;
  out.reserve(input_container.size());
  if constexpr(std::is_rvalue_reference_v<InputContainer&&>) {
    for(auto& input : input_container) {
      out.emplace_back(transform(std::move(input)));
    }
  } else {
    for(const auto& input : input_container) {
      out.emplace_back(transform(input));
    }
  }
  return out;
}

namespace details {
template <typename T, typename F, std::size_t... Is>
void tuple_for_each(T&& t, F f, std::index_sequence<Is...>) {
  (f(Is, std::get<Is>(std::forward<T>(t))), ...);
}

template <typename T, typename F, std::size_t... Is>
auto tuple_map(T&& t, F f, std::index_sequence<Is...>) {
  return std::make_tuple(f(Is, std::get<Is>(std::forward<T>(t)))...);
}

template <std::size_t Offset, typename... Ts, std::size_t... Is>
auto tuple_transform(std::tuple<Ts...> tuple, std::index_sequence<Is...>) {
  return std::make_tuple(std::get<Offset + Is>(std::move(tuple))...);
}
} // namespace details

template <typename... Ts, typename F, std::size_t... Is>
void tuple_for_each(const std::tuple<Ts...>& tuple, F f) {
  details::tuple_for_each(tuple, std::move(f),
                          std::make_index_sequence<sizeof...(Ts)>());
}

template <typename... Ts, typename F, std::size_t... Is>
void tuple_for_each(std::tuple<Ts...>&& tuple, F f) {
  details::tuple_for_each(std::move(tuple), std::move(f),
                          std::make_index_sequence<sizeof...(Ts)>());
}

template <typename... Ts, typename F, std::size_t... Is>
auto tuple_map(const std::tuple<Ts...>& tuple, F f) {
  return details::tuple_map(tuple, std::move(f),
                            std::make_index_sequence<sizeof...(Ts)>());
}

template <typename... Ts, typename F, std::size_t... Is>
auto tuple_map(std::tuple<Ts...>&& tuple, F f) {
  return details::tuple_map(std::move(tuple), std::move(f),
                            std::make_index_sequence<sizeof...(Ts)>());
}

template <typename T, typename... Ts>
std::tuple<Ts...> drop_first(std::tuple<T, Ts...> tuple) {
  return details::tuple_transform<1>(std::move(tuple),
                                     std::make_index_sequence<sizeof...(Ts)>());
}

static_assert(std::is_same_v<std::tuple<int>,
                             decltype(drop_first(std::make_tuple(1, 2)))>);

// template <>
// std::tuple<> drop_first(std::tuple<>) {
//   static_assert(false, "Cannot use drop first on empty tuple");
//   return std::make_tuple();
// }

} // namespace util
} // namespace anyf

#endif