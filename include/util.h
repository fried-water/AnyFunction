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
using small_vec3 = boost::container::small_vector<T, 3>;

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
  for(auto& input : input_container) {
    if constexpr(std::is_rvalue_reference_v<InputContainer&&>) {
      out.emplace_back(transform(std::move(input)));
    } else {
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

template <template <typename> typename Func, typename Tuple, typename Vec,
          std::size_t... Is>
Vec tuple_type_extraction_impl(std::index_sequence<Is...>) {
  Vec vec;
  vec.reserve(sizeof...(Is));
  (vec.emplace_back(Func<std::tuple_element_t<Is, Tuple>>::value), ...);

  return vec;
}

} // namespace details

template <template <typename> typename Func, typename Tuple, typename Vec>
Vec tuple_type_extraction() {
  return details::tuple_type_extraction_impl<Func, Tuple, Vec>(
      std::make_index_sequence<std::tuple_size_v<Tuple>>());
}

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

} // namespace util
} // namespace anyf

#endif