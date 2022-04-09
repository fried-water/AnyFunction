#pragma once

#include <boost/container/small_vector.hpp>

#include <any>
#include <functional>
#include <tuple>
#include <vector>

namespace anyf {

template <typename T, std::size_t N>
using SmallVec = boost::container::small_vector<T, N>;

template <typename T>
using SmallVecBase = boost::container::small_vector_base<T>;

struct Identity {
  template <typename T>
  T operator()(T t) const {
    return t;
  }
};

namespace util {

template <typename Vec, typename... Elements>
Vec make_vector(Elements&&... elements) {
  Vec vec;
  vec.reserve(sizeof...(Elements));
  (vec.emplace_back(std::forward<Elements>(elements)), ...);
  return vec;
}

template <typename T, typename... Elements>
auto make_std_vector(Elements&&... elements) {
  return make_vector<std::vector<T>>(std::forward<Elements>(elements)...);
}

template <typename T, std::size_t N, typename... Elements>
auto make_small_vector(Elements&&... elements) {
  return make_vector<SmallVec<T, N>>(std::forward<Elements>(elements)...);
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

template <typename TupleTypes, typename InputContainer, std::size_t... Is>
auto vec_to_tuple(InputContainer input, std::index_sequence<Is...>) {
  return std::make_tuple(
      std::any_cast<std::tuple_element_t<Is, TupleTypes>>(std::move(input[Is]))...);
}

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

template <typename TupleTypes, typename InputContainer>
auto vec_to_tuple(InputContainer input) {
  return details::vec_to_tuple<TupleTypes>(
      std::move(input), std::make_index_sequence<std::tuple_size_v<TupleTypes>>());
}

template <typename... Ts, typename F>
void tuple_for_each(const std::tuple<Ts...>& tuple, F f) {
  details::tuple_for_each(tuple, std::move(f), std::make_index_sequence<sizeof...(Ts)>());
}

template <typename... Ts, typename F>
void tuple_for_each(std::tuple<Ts...>&& tuple, F f) {
  details::tuple_for_each(std::move(tuple), std::move(f),
                          std::make_index_sequence<sizeof...(Ts)>());
}

template <typename... Ts, typename F, std::size_t... Is>
auto tuple_map(const std::tuple<Ts...>& tuple, F f) {
  return details::tuple_map(tuple, std::move(f), std::make_index_sequence<sizeof...(Ts)>());
}

template <typename... Ts, typename F, std::size_t... Is>
auto tuple_map(std::tuple<Ts...>&& tuple, F f) {
  return details::tuple_map(std::move(tuple), std::move(f),
                            std::make_index_sequence<sizeof...(Ts)>());
}

template <typename T, typename... Ts>
auto drop_first(std::tuple<T, Ts...> tuple) {
  return details::tuple_transform<1>(std::move(tuple), std::make_index_sequence<sizeof...(Ts)>());
}

} // namespace util
} // namespace anyf
