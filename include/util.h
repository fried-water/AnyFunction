#ifndef UTIL_H
#define UTIL_H

#include <boost/container/small_vector.hpp>

#include <any>
#include <tuple>
#include <functional>

namespace anyf {

template <typename T, std::size_t N>
using small_vec = boost::container::small_vector<T, N>;

template <typename T>
using small_vec_base = boost::container::small_vector_base<T>;

namespace util {

template <typename Vec, typename... Elements>
inline Vec make_vector(Elements&&... elements) {
  Vec vec;
  vec.reserve(sizeof...(Elements));
  (vec.emplace_back(std::forward<Elements>(elements)), ...);
  return vec;
}

template <typename Types, typename Vec, typename F, std::size_t... Is>
inline auto call_with_any_vec(F f, Vec inputs, std::index_sequence<Is...>) {
  return std::invoke(f, std::any_cast<std::tuple_element_t<Is, Types>>(std::move(inputs[Is]))...);
}

}
}

#endif