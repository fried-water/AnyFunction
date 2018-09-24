#ifndef UTIL_H
#define UTIL_H

#include <boost/container/small_vector.hpp>

namespace comp {

template <typename T, std::size_t N>
using small_vec = boost::container::small_vector<T, N>;

template <typename T>
using small_vec_base = boost::container::small_vector_base<T>;

namespace util {

template <typename T, std::size_t N, typename... Elements>
inline small_vec<T, N> create_small_vector(Elements&&... elements) {
  small_vec<T, N> vec;
  vec.reserve(sizeof...(Elements));
  (vec.emplace_back(std::forward<Elements>(elements)), ...);
  return vec;
}

}
}

#endif